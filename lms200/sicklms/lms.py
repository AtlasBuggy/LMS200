import time
import serial
import serial.tools.list_ports
import math
import numpy as np
from threading import Lock, Event

from atlasbuggy.datastream import ThreadedStream

from sicklms.data_structures import *
from sicklms.message import Message
from sicklms.constants import *


class SickLMS(ThreadedStream):
    """
    A class representing all functionality of a SICK LMS device
    """

    def __init__(self, port_address: str = None, enabled=True, log_level=None, name=None, baud=38400,
                 mode=SICK_OP_MODE_MONITOR_STREAM_VALUES, mode_params: bytes = None):
        """
        :param port_address: device address. For unix devices, this device will appear in /dev. 
            For windows, it will be a COM port. If None is given, this object will you a list to choose from
        :param baud: rate at which communicate with lms device. For the LMS200 the maximum value is 38400
        :param enabled: enabled flag
        :param name: name of the object (the class name by default)
        :param log_level: level of log messages to print
        :param mode: mode of operation. Stream values continuously by default
        :param mode_params: parameters for this mode. Check the "internal setters" section for these parameters
        """
        super(SickLMS, self).__init__(enabled, log_level, name)

        self.session_baud = baud
        if type(mode) == int and mode in sick_lms_operating_modes:
            self.session_mode = mode
        elif type(mode) == str and mode in sick_lms_operating_modes:
            self.session_mode = sick_lms_operating_modes[mode]
        else:
            self.session_mode = mode

        self.session_mode_params = mode_params

        self.device_path = port_address
        self.sick_type = SICK_LMS_TYPE_UNKNOWN
        self.operating_status = LmsOperatingStatus()  # object representing the operating status of the lms
        self.config = LmsConfig()  # object representing the configuration of the lms

        # easy access variables for important properties of the device
        self.scan_size = 0
        self.detection_angle_degrees = 0
        self.scan_resolution = 0
        self.distance_no_detection_mm = 50
        self.max_distance = 0.0
        self.update_rate = 0

        self.distances = np.array([])
        self.angles = np.array([])
        self.point_cloud = np.array([])

        self.subrange_start_index = 0
        self.subrange_stop_index = 0
        self.mean_sample_size = 0

        # objects representing the current scan of the lms
        self.current_scan = None
        self.b0_scan = ScanProfileB0()
        self.b6_scan = ScanProfileB6()
        self.b7_scan = ScanProfileB7()
        self.bf_scan = ScanProfileBF()
        self.c4_scan = ScanProfileC4()

        # pyserial variables
        self.serial_lock = Lock()
        self.serial_ref = None
        self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT

        # reusable object for capturing and parsing messages
        self.recv_container = Message()

        # internal flags
        self._config_has_updated = False
        self._has_initialized = Event()
        self.distance_log_tag = "[distance]"

    def start_up_commands(self):
        """
        Commands to run when the LMS start ups
        
        Override this method to access this behavior
        """
        pass

    def initialized(self):
        """
        Behavior to run when the LMS has initialized

        Override this method to access this behavior
        """
        pass

    def run(self):
        """
        Run behavior for the lms 
        """
        # note to self: never have threads trying to access the same serial reference.
        # Keep all serial activities inside or outside the thread. Not both

        self._startup_lms()  # initialize the device

        # run start up commands
        self.start_up_commands()
        self.initialized()

        # print LMS status
        print(self.status_string())
        self.logger.debug(self.status_string(one_line=True))

        # while running and if the operating mode is stream, get and parse point clouds
        while self.is_running():
            t0 = time.time()
            if self.operating_status.operating_mode == SICK_OP_MODE_MONITOR_STREAM_VALUES:
                self.get_scan()
                self._parse_point_cloud()
                self.point_cloud_received(self.point_cloud)
                self.logger.debug("%s %s" % (self.distance_log_tag, self.stringify_distance()))

                self.update_rate = 1 / (time.time() - t0)
            else:
                time.sleep(0.05)
        self.logger.debug("Loop exited")

    def stop(self):
        try:
            self._teardown()
        except SickIOException:
            self.logger.error("Encountered error while shutting down")

    # ----- point cloud creators -----

    def point_cloud_received(self, point_cloud):
        """
        Callback for receiving a point cloud.
        
        Override this method to access this behavior
        
        :param point_cloud: a numpy array containing the point cloud (all units in meters)
        """
        pass

    def _parse_point_cloud(self):
        """
        Convert distance and angle lists into a 2D point cloud using numpy operations
        """
        self.point_cloud = np.vstack(
            [self.distances * np.cos(self.angles), self.distances * np.sin(self.angles)]).T

    def _make_distances(self):
        """
        Convert the current scan into the correct format and units (meters)
        """
        self.distances = np.array(self.current_scan.measurements[:self.current_scan.num_measurements], dtype=np.float32)

        if self.operating_status.measuring_units == SICK_MEASURING_UNITS_CM:
            conversion = 0.01
        else:
            conversion = 0.001
        self.distances *= conversion
        # self.distances[self.distances > self.max_distance] = 0.0  # TODO: causes SIGBUS errors...

    def _make_angles(self):
        """
        Create angles list in the correct format and units (radians)
        """
        self.detection_angle_degrees = self.operating_status.scan_angle
        self.scan_resolution = self.operating_status.scan_resolution * 0.01

        scan_angle_radians = math.radians(self.detection_angle_degrees)
        resolution_radians = math.radians(self.scan_resolution)

        self.angles = np.arange(0, scan_angle_radians + resolution_radians, resolution_radians)
        self.scan_size = len(self.angles)

    # ----- internal start up methods -----

    def _startup_lms(self):
        """
        Start up behavior for the lms
        """

        # setup pyserial connection
        self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT
        self._setup_connection()

        # attempt to connect with the default session baud rate
        session_baud_set = False
        try:
            self._set_session_baud(self.session_baud)
            self.logger.debug("Baud successfully set")
            session_baud_set = True
        except (SickTimeoutException, SickIOException):
            self.logger.warning("Failed to set requested baud rate (%s). "
                                "Attempting to detect LMS baud rate... Please wait while "
                                "a connection is attempted. This may take some time." % self.session_baud)

        # if initial baud rate failed, try to discover it by trying one of the four options
        if not session_baud_set:
            discovered_baud = None
            for test_baud in sick_lms_baud_codes.keys():
                self.logger.info("Trying baud: %s" % test_baud)

                if self._test_baud(test_baud):
                    discovered_baud = test_baud
                    break
                self.serial_ref.flush()

            if discovered_baud is None:
                self.logger.warning("Failed to detect baud rate!")
                raise SickIOException("Failed to detect baud rate!")

            self.logger.info("Setting baud to: %s" % discovered_baud)
            self._set_session_baud(discovered_baud)

        # obtain the lms configuration
        self._get_type()
        self._get_status()
        self._get_config()

        # change to the desired operation mode
        self._switch_opmode(self.session_mode, self.session_mode_params)

        self._has_initialized.set()

    def _teardown(self):
        """
        Teardown behavior for the lms
        """

        self.logger.debug("Tearing down connection")

        # only proceed if the connection was opened
        if self._is_open():

            # only proceed if the device was initialized successfully
            if self._has_initialized.is_set():
                try:
                    # change to the request mode (in this mode the device won't return data unless asked)
                    self.logger.debug("Switching to monitor request mode")
                    self._set_opmode_monitor_request()

                    self.logger.debug("Changing to default baud")
                    self._set_session_baud(DEFAULT_SICK_LMS_SICK_BAUD)
                except BaseException:
                    # if anything goes wrong, close the serial connection
                    self.logger.error("Error encountered while tearing down")
                    self._teardown_connection()
                    raise
            else:
                self.logger.warning("Never initialized. Skipping teardown")
            self._teardown_connection()
        else:
            self.logger.debug("Connection was not open!")

    def _setup_connection(self):
        """
        Auto discover the lms serial port
        """
        if self.device_path is None:
            self.device_path = self._auto_find_port()

        # initialize pyserial instance
        self.serial_ref = serial.Serial(
            port=self.device_path,
            baudrate=DEFAULT_SICK_LMS_SICK_BAUD,
            timeout=DEFAULT_SICK_LMS_SICK_MAX_TIMEOUT
        )

    def _auto_find_port(self):
        ports = []
        for port_no, description, address in serial.tools.list_ports.comports():
            self.logger.debug("Discovered port: %s: '%s', '%s'" % (port_no, description, address))
            if 'USB' in address:
                ports.append((port_no, description, address))

        if len(ports) == 0:
            self.logger.error("Exception while auto detecting port")
            raise SickIOException("No USB ports available...")

        if len(ports) == 1:
            return ports[0][0]

        print("Select a discovered port:")
        for index, (port_no, description, address) in enumerate(ports):
            print("[%s] %s: '%s', '%s'" % (index, port_no, description, address))
        selected_index = None
        while selected_index is None:
            try:
                value = int(input(">> "))
                if 0 <= value < len(ports):
                    selected_index = value
                else:
                    print("That value isn't between 0 and %s!" % len(ports))
            except ValueError:
                print("Input is not an integer!")

        return ports[selected_index][0]

    def _teardown_connection(self):
        """
        Close the serial reference if it's open
        """
        if self._is_open():
            self.serial_ref.close()

    def _is_open(self):
        """
        Check if the serial reference has been opened
        """
        return self.serial_ref is not None and self.serial_ref.isOpen()

    def _set_session_baud(self, baud):
        """
        Set the device's baud to the session's baud
        
        :param baud: of four baud rates
        """
        self.session_baud = baud

        # check if the input baud is valid
        if baud not in sick_lms_baud_codes:
            raise SickConfigException("Invalid baud rate: %s" % baud)

        # construct the message to send
        payload = b'\x20'
        payload += Message.int_to_byte(sick_lms_baud_codes[baud], 1)
        self.logger.debug("Changing baud to: %s" % baud)

        # send the message
        self._send_message_and_get_reply(Message.make(payload))

        # set the pyserial reference baud
        self._set_terminal_baud(baud)

        # lms likes a sleep here
        time.sleep(0.25)

        # flush the serial buffer
        self.serial_ref.flush()

    def _test_baud(self, baud):
        """
        Test if the lms device is at the input baud rate 
        
        :param baud: baud rate to test
        """
        self._set_terminal_baud(baud)

        attempts = 0

        # try 3 times to obtain a message
        while True:
            try:
                attempts += 1
                self._get_errors()
                return True

            except SickIOException:
                if attempts > DEFAULT_SICK_LMS_NUM_TRIES:
                    self.logger.error("Encountered errors while testing baud. Max attempts reached")
                    return False
            except (SickTimeoutException, SickConfigException):
                self.logger.exception("Encountered errors while testing baud")
                return False

    # ----- user exposed setters -----

    def set_range(self, depth):
        """
        Set the lms device's range to 8, 16, or 32 meters
        
        :param depth: 8, 16, or 32
        """
        self.logger.debug("Setting measuring parameters: %s" % depth)
        if depth <= 8:
            self.set_measuring_units(SICK_MEASURING_UNITS_MM)
            self.set_measuring_mode(SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE)
        elif 8 < depth <= 16:
            self.set_measuring_units(SICK_MEASURING_UNITS_MM)
            self.set_measuring_mode(SICK_MS_MODE_16_FA_FB)
        elif 16 < depth:
            self.set_measuring_units(SICK_MEASURING_UNITS_CM)
            self.set_measuring_mode(SICK_MS_MODE_32_FA)
        else:
            return

        # update configuration with these parameters
        self.update_config()

    def set_resolution(self, scan_angle, scan_resolution):
        """
        Set scan properties
        
        :param scan_angle: 90, 100, or 180 (degrees)
        :param scan_resolution: 0.25, 0.50, 1.00 (degrees)
        :return: 
        """
        if self.sick_type == SICK_LMS_TYPE_211_S14 or \
                        self.sick_type == SICK_LMS_TYPE_221_S14 or \
                        self.sick_type == SICK_LMS_TYPE_291_S14:
            raise SickConfigException("Command not supported on this model!")

        # check if the scan angle is valid
        if not is_valid_scan_angle(scan_angle):
            raise SickConfigException("Undefined scan angle: %s" % repr(scan_angle))

        # check if the scan resolution is valid
        scan_resolution = int(scan_resolution * 100)
        if not is_valid_scan_resolution(scan_resolution):
            raise SickConfigException("Undefined scan resolution: %s" % repr(scan_resolution))

        # check any values actually changed
        if self.operating_status.scan_angle != scan_angle or self.operating_status.scan_resolution != scan_resolution:
            self.logger.debug("Setting variant. Angle: %s, resolution: %s" % (scan_angle, scan_resolution))

            # create the message
            payload = b'\x3b'
            payload += sick_lms_scan_angles[scan_angle]
            payload += sick_lms_scan_resolutions[scan_resolution]

            # lms stops sending data if the variant (angle and resolution) is reset midstream. This prevents errors
            self._set_opmode_monitor_request()

            # send the message
            response = self._send_message_and_get_reply(Message.make(payload))
            if response.payload[1] != 0x01:
                raise SickConfigException("Configuration was unsuccessful!")

            # parse the response and update important variables
            self.operating_status.scan_angle = response.parse_int(2, 2)
            self.operating_status.scan_resolution = response.parse_int(4, 2)
            self._make_angles()

            # return lms to the original operating mode
            self._switch_opmode(self.session_mode, self.session_mode_params)

    def set_measuring_units(self, units=SICK_MEASURING_UNITS_MM):
        """
        Switch the device units. Call update_config after this
        
        :param units: SICK_MEASURING_UNITS_MM or SICK_MEASURING_UNITS_CM
        """
        if is_valid_measuring_units(units) and units != self.config.measuring_units:
            self.logger.debug("Setting units: %s" % sick_lms_measuring_units[units])
            self.config.measuring_units = units
            self._config_has_updated = True

    def set_sensitivity(self, sensitivity=SICK_SENSITIVITY_STANDARD):
        """
        Change the device sensitivity. Call update_config after this
        
        :param sensitivity: A sensitivity value. Check constants.py for all of them
        """
        if not is_valid_sensitivity(sensitivity):
            raise SickConfigException("Invalid sensitivity input: %s" % repr(sensitivity))

        if sensitivity != self.config.peak_threshold:
            self.logger.debug("Setting sensitivity: %s -> %s" % (self.config.peak_threshold, sensitivity))
            self.config.peak_threshold = sensitivity
            self._config_has_updated = True

    def set_measuring_mode(self, measuring_mode=SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE):
        """
        Change the device measuring mode. Call update_config after this

        :param measuring_mode: A measuring mode value. Check constants.py for all of them
        """
        if not is_valid_measuring_mode(measuring_mode):
            raise SickConfigException("Undefined measuring mode: '%s'" % repr(measuring_mode))

        if measuring_mode != self.config.measuring_mode:
            self.logger.debug("Setting measuring mode: %s -> %s" % (self.config.measuring_mode, measuring_mode))
            self.config.measuring_mode = measuring_mode
            self._config_has_updated = True

        self.max_distance = get_max_distance(measuring_mode)

    def set_availability(self, availability_flag=SICK_FLAG_AVAILABILITY_DEFAULT):
        """
        Change the device availability. Call update_config after this

        :param availability: An availability value. Check constants.py for all of them
        """
        if availability_flag > 7:
            raise SickConfigException("Invalid availability: %s" % repr(availability_flag))

        if availability_flag != self.config.availability_level:
            self.logger.debug("Setting availability: %s -> %s" % (self.config.availability_level, availability_flag))
            # Maintain the higher level bits
            self.config.availability_level &= 0xf8

            # Set the new availability flags
            self.config.availability_level |= availability_flag

            self._config_has_updated = True

    # ----- data getters -----

    def get_scan(self, reflect_values=False):
        """
        Set the operating mode to stream if the device isn't already
        
        :param reflect_values: Return reflect values instead of distances
        """

        # switch operating mode if the device isn't in the correct mode
        self._set_opmode_monitor_stream()

        # receive a scan
        response = self._recv_message()

        if reflect_values:
            # parse scan with c4_scan if reflect_values is True
            if response.payload[0] != 0xc4:
                raise SickIOException("Invalid response for current measurement mode: %s" % response)
            self.c4_scan.parse_scan_profile(response.payload[1:], self.operating_status.measuring_mode)
            self.current_scan = self.c4_scan
        else:
            # parse scan as a normal distance scan
            if response.payload[0] != 0xb0:
                raise SickIOException("Invalid response for current measurement mode: %s" % response)
            self.b0_scan.parse_scan_profile(response.payload[1:], self.operating_status.measuring_mode)
            self.current_scan = self.b0_scan

            # parse distance values
            self._make_distances()

    def get_scan_subrange(self, start_index, stop_index):
        """
        Get and parse a subrange scan
        
        :param start_index: Scan start index
        :param stop_index: Scan stop index
        """

        # switch operating mode if the device isn't in the correct mode
        self._set_opmode_monitor_stream_subrange(start_index, stop_index)

        # receive a scan
        response = self._recv_message()

        if response.payload[0] != 0xb7:
            raise SickIOException("Invalid response for current measurement mode: %s" % response)

        # parse scan
        self.b7_scan.parse_scan_profile(response.payload[1:], self.operating_status.measuring_mode)
        self.current_scan = self.b7_scan
        self._make_distances()

    def get_partial_scan(self):
        """
        Get and parse a partial scan
        """
        # switch operating mode if the device isn't in the correct mode
        self._set_opmode_monitor_stream_partial_scan()

        # receive a scan
        response = self._recv_message()

        if response.payload[0] != 0xb0:
            raise SickIOException("Invalid response for current measurement mode: %s" % response)

        # parse scan
        self.b0_scan.parse_scan_profile(response.payload[1:], self.config.measuring_mode)
        self.current_scan = self.b0_scan
        self._make_distances()

    def get_mean_values(self, sample_size):
        """
        Get and parse a mean scan
        
        :param sample_size: Number of samples to take for the mean
        """
        # switch operating mode if the device isn't in the correct mode
        self._set_opmode_monitor_stream_mean(sample_size)

        # receive a scan
        response = self._recv_message()

        if response.payload[0] != 0xb6:
            raise SickIOException("Invalid response for current measurement mode: %s" % response)

        # parse scan
        self.b6_scan.parse_scan_profile(response.payload[1:], self.config.measuring_mode)
        self.current_scan = self.b6_scan
        self._make_distances()

    def get_mean_values_subrange(self, sample_size, start_index, stop_index):
        """
        Get and parse a mean subrange scan
        
        :param sample_size: Number of samples to take for the mean
        :param start_index: Scan start index
        :param stop_index: Scan end index
        """

        # switch operating mode if the device isn't in the correct mode
        self._set_opmode_monitor_stream_mean_subrange(sample_size, start_index, stop_index)

        # receive a scan
        response = self._recv_message()
        if response.payload[0] != 0xb7:
            raise SickIOException("Invalid response for current measurement mode: %s" % response)

        # parse scan
        self.b7_scan.parse_scan_profile(response.payload[1:], self.config.measuring_mode)
        self.current_scan = self.b7_scan
        self._make_distances()

    # ----- internal getters -----
    def _get_errors(self):
        """
        Ask the lms device for any errors it might have
        """
        self.logger.debug("Getting errors")

        # create and send the message
        response = self._send_message_and_get_reply(Message.make(b'\x32'))

        # parse error message
        k = 1
        num_errors = int((len(response.payload) - 2) / 2)
        error_type_buffer = b''
        error_num_buffer = b''
        for index in range(num_errors):
            error_type_buffer += response.payload[k:k + 1]
            k += 1

            error_num_buffer += response.payload[k:k + 1]
            k += 1

        if len(error_num_buffer) > 0 or len(error_num_buffer) > 0:
            self.logger.error("Encountered errors: %s, %s" % (error_type_buffer, error_num_buffer))

    def _get_status(self):
        """
        Get the lms device status
        """
        self.logger.debug("Getting status")

        # create and send message
        response = self._send_message_and_get_reply(Message.make(b'\x31'))

        # parse response
        try:
            self.operating_status.parse_status(response)
        except IndexError as error:
            raise RuntimeError("Invalid response: %s" % response) from error

        self.logger.debug(
            "Scan angle: %s, resolution: %s" % (
                self.operating_status.scan_angle, self.operating_status.scan_resolution)
        )

        self._make_angles()

    def _get_config(self):
        """
        Get the lms device configuration
        """
        self.logger.debug("Getting configuration")

        # create and send message
        response = self._send_message_and_get_reply(Message.make(b'\x74'))

        # parse response
        self.config.parse_config_profile(response)

    def _get_type(self):
        """
        Get which type this lms device is
        """

        # create and send message
        response = self._send_message_and_get_reply(Message.make(b'\x3a'))

        # parse response
        status = 0
        model_string = response.parse_string(1).split(";")
        for key, value in supported_models.items():
            # parse each part of the model type and check it against all known types
            self.sick_type = SICK_LMS_TYPE_UNKNOWN
            if type(key) == str:
                model = key.split(";")

                if model_string[0] == model[0] and model_string[1] == model[1]:
                    # device matches exactly. No more checks required
                    self.sick_type = value
                    self.logger.debug("LMS is of type: %s" % model_string)
                    break
                elif model_string[0] == model[0] and model_string[1] != model[1]:
                    # device type matches but software version does not
                    status = 1
                    self.sick_type = value
                else:
                    # don't override value if a close match has already been found
                    if status != 1:
                        # neither device type nor software match
                        status = 2

        # only print a message once all conditions have been checked
        if status == 1:
            self.logger.debug("Device types match but software version is not supported")
        elif status == 2:
            self.logger.warning("LMS is of an unknown type: %s" % model_string)

    # ----- user exposed lms functionality -----
    def update_config(self):
        """Update device configuration """

        # only update the config if values have actually been changed
        if self._config_has_updated:

            # set to installation mode
            self._set_opmode_installation()

            # create the config message (currently only measuring mode and units are supported)
            message = self.config.build_message()

            # send config message
            self.logger.debug("Setting new configuration")
            response = self._send_message_and_get_reply(message)

            # check if the response is correct
            if response.payload[1] != 0x01:
                raise SickConfigException("Configuration failed!")

            # wait for the configuration to propagate
            time.sleep(0.25)
            self.logger.debug("Configuration set! Switching back to mode: %s" %
                              sick_lms_operating_modes[self.session_mode])
            # get the new device status
            self._get_status()

            # switch back to original operating mode
            self._switch_opmode(self.session_mode, self.session_mode_params)

            # flip the config flag back
            self._config_has_updated = False
        else:
            self.logger.debug("Current configuration unchanged!")

    def reset(self):
        """Reset the lms device """

        # resets can take some time, set the timeout to a minute
        self._timeout = 60
        self.logger.debug("Resetting...")

        # create and send the reset message. Expect a reply code of 0x91
        self._send_message_and_get_reply(Message.make(b'\x10'), reply_code=0x91)

        # set to the default baud
        self._set_terminal_baud(DEFAULT_SICK_LMS_SICK_BAUD)

        self.logger.debug("Back online!")

        # receive reset message
        self._timeout = 30
        try:
            self._recv_message(reply_code=0x90)
        except SickIOException as error:
            self.logger.exception(error)
            self.logger.warning("Unexpected reply! (assuming device has been reset)")

        self.logger.debug("Starting back up")

        # start up the lms as normal
        self._startup_lms()

    # ----- internal setters -----
    def _set_opmode_installation(self):
        """Set to installation mode"""
        old_timeout = self._timeout
        self._timeout = DEFAULT_SICK_LMS_SICK_CONFIG_MESSAGE_TIMEOUT
        self._switch_opmode(SICK_OP_MODE_INSTALLATION, DEFAULT_SICK_LMS_SICK_PASSWORD)
        self._timeout = old_timeout
        self.logger.debug("Successfully entered installation mode")

    def _set_opmode_diagnostic(self):
        """Set to diagnostic mode"""
        self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT
        self._switch_opmode(SICK_OP_MODE_DIAGNOSTIC)

    def _set_opmode_monitor_request(self):
        """Set to monitor request mode"""
        self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT
        self._switch_opmode(SICK_OP_MODE_MONITOR_REQUEST_VALUES)

    def _set_opmode_monitor_stream(self):
        """Set to monitor stream mode"""
        self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT
        self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_VALUES)

    def _set_opmode_monitor_stream_range_reflectivity(self):
        """Set to stream reflectivity mode"""
        self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT
        self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT, [0x01, 0x00, 0xB5, 0x00])

    def _set_opmode_monitor_stream_partial_scan(self):
        """Set to partial stream mode"""
        self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT
        self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN)

    def _set_opmode_monitor_stream_mean(self, sample_size):
        """
        Set to stream mean values mode
        
        :param sample_size: number of samples to use for the mean 
        """
        if not (2 < sample_size < 250):
            raise SickConfigException("Invalid sample size: %s" % repr(sample_size))
        if self.mean_sample_size != sample_size:
            self.mean_sample_size = sample_size

            mode_params = Message.int_to_byte(sample_size, 1)

            self._timeout = DEFAULT_SICK_LMS_SICK_MEAN_VALUES_MESSAGE_TIMEOUT
            self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN, mode_params)

    def _set_opmode_monitor_stream_subrange(self, start_index, stop_index):
        """
        Set to stream subrange mode
        
        :param start_index: Scan start index
        :param stop_index: Scan stop index
        """
        if self.subrange_start_index != start_index or self.subrange_stop_index != stop_index:
            max_subrange_stop_index = self.operating_status.scan_angle * 100 / self.operating_status.scan_resolution + 1
            if start_index > stop_index or start_index == 0 or stop_index > max_subrange_stop_index:
                raise SickConfigException("Invalid subregion bounds")

            self.subrange_start_index = start_index
            self.subrange_stop_index = stop_index

            mode_params = Message.int_to_byte(start_index, 2)
            mode_params += Message.int_to_byte(stop_index, 2)

            self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT
            self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE, mode_params)

    def _set_opmode_monitor_stream_mean_subrange(self, sample_size, start_index, stop_index):
        """
        Set to stream subrange mean values mode

        :param sample_size: number of samples to use for the mean 
        :param start_index: Scan start index
        :param stop_index: Scan stop index
        """
        if self.subrange_start_index != start_index or \
                        self.subrange_stop_index != stop_index or \
                        self.mean_sample_size != sample_size:
            if not (2 < sample_size < 250):
                raise SickConfigException("Invalid sample size: %s" % repr(sample_size))

            max_subrange_stop_index = self.operating_status.scan_angle * 100 / self.operating_status.scan_resolution + 1
            if start_index > stop_index or start_index == 0 or stop_index > max_subrange_stop_index:
                raise SickConfigException("Invalid subregion bounds")

            self.mean_sample_size = sample_size
            self.subrange_start_index = start_index
            self.subrange_stop_index = stop_index

            mode_params = Message.int_to_byte(sample_size, 1)
            mode_params += Message.int_to_byte(start_index, 2)
            mode_params += Message.int_to_byte(stop_index, 2)

            self._timeout = DEFAULT_SICK_LMS_SICK_MEAN_VALUES_MESSAGE_TIMEOUT
            self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE, mode_params)

    def _switch_opmode(self, mode: int, mode_params: bytes = None):
        """
        Switch to the input mode
        
        :param mode: A valid mode code
        :param mode_params: additional bytes to send
        """
        if self.operating_status.operating_mode == mode:
            return

        payload_buffer = b'\x20'
        payload_buffer += Message.int_to_byte(mode, 1)

        if mode_params is not None:
            payload_buffer += mode_params

        self.logger.debug("Swiching to operating mode: %s" % sick_lms_operating_modes[mode])
        response = self._send_message_and_get_reply(Message.make(payload_buffer))
        if len(response.payload) < 2 or response.payload[1] != 0x00:
            raise SickConfigException("configuration request failed! Here's the response: %s" % response)

        self.operating_status.operating_mode = mode

    def _set_terminal_baud(self, baud):
        """
        Set the pyserial reference baud rate
        
        :param baud: the new baud rate
        """
        self.serial_ref.baudrate = baud

    # ----- telegram transfer and protocol methods -----

    def _send_message_and_get_reply(self, message: bytes, num_tries=DEFAULT_SICK_LMS_NUM_TRIES,
                                    reply_code: int = None):
        """
        Send a message and block until a response is received
        
        :param message: bytes to send
        :param num_tries: Number of attempts
        :param reply_code: The expected first byte of the response
        """
        for attempt in range(num_tries):
            try:
                self._send_message(message)
                return self._recv_message(reply_code)

            except SickTimeoutException as error:
                self.logger.exception(error)

                if attempt == num_tries - 1:
                    raise

            except BaseException as error:
                self.logger.exception(error)
                raise

        raise SickIOException("Maximum of attempts reached for message: %s" % repr(message))

    def _send_message(self, message: bytes):
        """
        Send bytes to the lms device. Check for the acknowledgment byte (ACK) or no acknowledgement (NACK)
        
        :param message: bytes to send
        """
        self.serial_ref.write(message)
        if not self._check_for_ack():
            raise SickIOException("Command not received correctly!")

    def _recv_message(self, reply_code: int = None) -> Message:
        """
        Wait for a response that starts with b'\x02\x80'
        
        :param reply_code: The expected first byte of the payload
        :return: The message response
        """
        satisfied = False
        t0 = time.time()

        while not satisfied:
            if time.time() - t0 > self._timeout:
                raise SickTimeoutException("Read timed out!!")

            # reset the container buffer
            self.recv_container.reset()

            # look for the message header
            if self._read_8() == DEFAULT_SICK_LMS_MESSAGE_HEADER:
                if self._read_8() == DEFAULT_SICK_LMS_HOST_ADDRESS:
                    satisfied = True

        response = self._parse_response()
        if reply_code is not None and response.code != reply_code:
            raise SickIOException("Reply codes don't match (expected: '%s', got '%s')" % (
                reply_code, response.code))

        return response

    def _parse_response(self):
        """If b'\x02\x80' has been received, parse the rest of the characters on the serial buffer"""

        length = self._read_16()
        payload = self._read(length)
        checksum = self._read_16()

        self.logger.debug("Got message: length = %s, payload = %s, checksum = %s" % (length, payload, checksum))

        # parse message from response
        self.recv_container.make_message(length, payload, checksum)
        return self.recv_container

    def _check_for_ack(self):
        """Wait for ACK or NACK byte"""
        ack = b'\x00'
        responses = b''
        attempts = 0
        t0 = time.time()
        while ack == b'\x00' or attempts < DEFAULT_SICK_LMS_NUM_TRIES:
            ack = self.serial_ref.read(1)
            if len(ack) > 0:
                responses += ack
                if ack == b'\x06':
                    return True
                elif ack == b'\x15':
                    return False
            else:
                self.logger.debug("Invalid ACK check. Nothing received!!")
                return False

            if time.time() - t0 > self._timeout:
                attempts += 1
                t0 = time.time()
                self.logger.debug("ack check timed out. Attempt %s of %s" % (attempts, DEFAULT_SICK_LMS_NUM_TRIES))

            if attempts > DEFAULT_SICK_LMS_NUM_TRIES:
                raise SickTimeoutException("Failed to receive ACK. Timed out!!")

        try:
            message = "Invalid value received (neither ACK, nor NACK): %s" % responses
            self.logger.debug(message)
            raise SickIOException(message)
        except SickIOException as error:
            self.logger.exception(error)
            raise

    # ----- pyserial interfaces -----

    def _read_with_timeout(self, n):
        """Read n characters with a timeout"""
        t0 = time.time()
        response = b''
        while time.time() - t0 < self._timeout:
            response += self.serial_ref.read(n)
            if len(response) == n:
                break
        if time.time() - t0 > self._timeout:
            self.logger.debug("Serial read timed out!")
            raise SickTimeoutException("Serial read timed out!")

        return response

    def _read(self, num_bytes) -> bytes:
        """Safely read serial and append the read characters to the receive buffer"""
        with self.serial_lock:
            result = b''
            if self._is_open():
                result = self._read_with_timeout(num_bytes)
            else:
                raise SickIOException("Serial not open for reading!")

        if len(result) == 0:
            raise SickTimeoutException("Failed to read serial!")

        self.recv_container.append_byte(result)
        return result

    def _read_8(self) -> int:
        """Read one character and parse it as an 8 bit integer"""
        return ord(self._read(1))

    def _read_16(self) -> int:
        """
        Read two characters and parse it as a 16 bit integer.
        The first byte received is the lower byte.
        """
        lower_byte = self._read_8()
        upper_byte = self._read_8()

        return (upper_byte << 8) + lower_byte

    # ----- device type checks -----

    def _is_lms200(self):
        return supported_models[self.sick_type][3:6] == "200"

    def _is_lms211(self):
        return supported_models[self.sick_type][3:6] == "211"

    def _is_lms220(self):
        return supported_models[self.sick_type][3:6] == "220"

    def _is_lms221(self):
        return supported_models[self.sick_type][3:6] == "221"

    def _is_lms291(self):
        return supported_models[self.sick_type][3:6] == "291"

    def _is_unknown(self):
        return self.sick_type == SICK_LMS_TYPE_UNKNOWN

    # ----- status string methods -----

    def stringify_distance(self):
        return " ".join([str(distance) for distance in self.distances])

    def status_string(self, one_line=False):
        string = "'%s' status:\n" \
                 "\tVariant: %s\n" \
                 "\tType: %s\n" \
                 "\tSensor status: %s\n" \
                 "\tScan angle: %sº\n" \
                 "\tScan resolution: %sº\n" \
                 "\tOperating mode: %s\n" \
                 "\tMeasuring mode: %s\n" \
                 "\tMeasuring units: %s\n" % (
                     self.name, variant_to_string(self.operating_status.variant), supported_models[self.sick_type],
                     status_to_string(self.operating_status.device_status), self.operating_status.scan_angle,
                     self.operating_status.scan_resolution * 0.01,
                     sick_lms_operating_modes[self.operating_status.operating_mode],
                     sick_lms_measuring_modes[self.operating_status.measuring_mode],
                     sick_lms_measuring_units[self.operating_status.measuring_units]
                 )
        if one_line:
            string = string.replace("\n", "; ")
        return string

    def __str__(self):
        return self.status_string(one_line=True)
