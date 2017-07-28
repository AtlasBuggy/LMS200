import time
import serial
import serial.tools.list_ports
import math
import numpy as np
from threading import Lock

from atlasbuggy.datastream import ThreadedStream

from sicklms.data_structures import *
from sicklms.message import Message
from sicklms.constants import *


class SickLMS(ThreadedStream):
    def __init__(self, port_address=None, baud=38400, enabled=True, name=None, log_level=None,
                 mode=SICK_OP_MODE_MONITOR_STREAM_VALUES, mode_params=None):
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
        self.operating_status = LmsOperatingStatus()
        self.config = LmsConfig()

        self.scan_size = 0
        self.detection_angle_degrees = 0
        self.scan_resolution = 0
        self.distance_no_detection_mm = 50

        self.subrange_start_index = 0
        self.subrange_stop_index = 0
        self.mean_sample_size = 0

        self.current_scan = None
        self.b0_scan = ScanProfileB0()
        self.b6_scan = ScanProfileB6()
        self.b7_scan = ScanProfileB7()
        self.bf_scan = ScanProfileBF()
        self.c4_scan = ScanProfileC4()

        self.distances = np.array([])
        self.angles = np.array([])
        self.point_cloud = np.array([])

        self.serial_lock = Lock()
        self.serial_ref = None
        self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT

        self.recv_message = Message()
        self.send_message = Message()

        self.config_has_updated = False

        self.update_rate = 0

        self.distance_log_tag = "[distance]"

    def start_up_commands(self):
        pass

    def initialized(self):
        pass

    def run(self):
        # note to self: never have threads trying to access the same serial reference.
        # Keep all serial activities inside or outside the thread. Not both
        self._startup_lms()
        self.start_up_commands()
        self.initialized()

        print(self.status_string())
        self.logger.debug(self.status_string(one_line=True))

        while self.running():
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
        pass

    def _parse_point_cloud(self):
        self.point_cloud = np.vstack(
            [self.distances * np.cos(self.angles), self.distances * np.sin(self.angles)]).T

    def _make_distances(self):
        self.distances = np.array(self.current_scan.measurements[:self.current_scan.num_measurements], dtype=np.float32)

        if self.operating_status.measuring_units == SICK_MEASURING_UNITS_CM:
            conversion = 0.01
        else:
            conversion = 0.001
        self.distances *= conversion

    def _make_angles(self):
        self.detection_angle_degrees = self.operating_status.scan_angle
        self.scan_resolution = self.operating_status.scan_resolution * 0.01

        scan_angle_radians = math.radians(self.detection_angle_degrees)
        resolution_radians = math.radians(self.scan_resolution)

        self.angles = np.arange(0, scan_angle_radians + resolution_radians, resolution_radians)
        self.scan_size = len(self.angles)

    # ----- internal start up methods -----

    def _startup_lms(self):
        self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT
        self._setup_connection()

        session_baud_set = False
        try:
            self._set_session_baud(self.session_baud)
            self.logger.debug("Baud successfully set")
            session_baud_set = True
        except (SickTimeoutException, SickIOException):
            self.logger.debug("Failed to set requested baud rate (%s). "
                              "Attempting to detect LMS baud rate..." % self.session_baud)

        if not session_baud_set:
            discovered_baud = None
            for test_baud in sick_lms_baud_codes.keys():
                self.logger.debug("Trying baud: %s" % test_baud)

                if self._test_baud(test_baud):
                    discovered_baud = test_baud
                    break

            if discovered_baud is None:
                self.logger.error("Failed to detect baud rate!")
                raise SickIOException("Failed to detect baud rate!")

            self.logger.debug("Setting baud to: %s" % discovered_baud)
            self._set_session_baud(discovered_baud)

        self._get_type()
        self._get_status()
        self._get_config()

        self._switch_opmode(self.session_mode, self.session_mode_params)

    def _teardown(self):
        self.logger.debug("Tearing down connection")
        if self._is_open():
            try:
                self.logger.debug("Switching to monitor request mode")
                self._set_opmode_monitor_request()

                self.logger.debug("Changing to default baud")
                self._set_session_baud(DEFAULT_SICK_LMS_SICK_BAUD)
            except BaseException:
                self.logger.exception("Error encountered while tearing down")
                self._teardown_connection()
                raise
            self._teardown_connection()
        else:
            self.logger.debug("Connection was not open!")

    def _setup_connection(self):
        if self.device_path is None:
            self._auto_find_port()
        try:
            self.serial_ref = serial.Serial(
                port=self.device_path,
                baudrate=DEFAULT_SICK_LMS_SICK_BAUD,
                timeout=DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT
            )
        except serial.SerialException as error:
            status = self._auto_find_port()
            if status:
                self.logger.exception("Exception while auto detecting port")
                raise status from error

    def _auto_find_port(self):
        ports = []
        for port_no, description, address in serial.tools.list_ports.comports():
            self.logger.debug("Discovered port: %s: '%s', '%s'" % (port_no, description, address))
            if 'USB' in address:
                ports.append((port_no, description, address))

        if len(ports) == 0:
            return SickIOException("No USB ports available...")

        print("Select a discovered port:")
        for index, (port_no, description, address) in enumerate(ports):
            print("[%s] %s: '%s', '%s'" % (index, port_no, description, address))
        selected_index = None
        while selected_index is None:
            try:
                selected_index = int(input(">> "))
            except ValueError:
                print("Input is not an integer!")

    def _teardown_connection(self):
        if self._is_open():
            self.serial_ref.close()

    def _is_open(self):
        return self.serial_ref is not None and self.serial_ref.isOpen()

    def _set_session_baud(self, baud):
        self.session_baud = baud

        if baud not in sick_lms_baud_codes:
            raise SickConfigException("Invalid baud rate: %s" % baud)
        payload = b'\x20'
        payload += Message.int_to_byte(sick_lms_baud_codes[baud], 1)
        self.send_message.payload = payload

        self.logger.debug("Changing baud to: %s" % baud)
        self._send_message_and_get_reply(self.send_message.make_buffer())
        self._set_terminal_baud(baud)
        time.sleep(0.25)
        self.serial_ref.flush()

    def _test_baud(self, baud):
        self._set_terminal_baud(baud)

        attempts = 0

        while True:
            try:
                attempts += 1
                self._get_errors()
                return True

            except SickIOException as error:
                if attempts > DEFAULT_SICK_LMS_NUM_TRIES:
                    self.logger.exception("Encountered errors while testing baud. Max attempts reached")
                    raise error
            except (SickTimeoutException, SickConfigException):
                self.logger.exception("Encountered errors while testing baud")
                return False

    # ----- user exposed setters -----

    def set_range(self, depth):
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

        self.update_config()

    def set_resolution(self, scan_angle, scan_resolution):
        if self.sick_type == SICK_LMS_TYPE_211_S14 or \
                        self.sick_type == SICK_LMS_TYPE_221_S14 or \
                        self.sick_type == SICK_LMS_TYPE_291_S14:
            raise SickConfigException("Command not supported on this model!")

        if not is_valid_scan_angle(scan_angle):
            raise SickConfigException("Undefined scan angle: %s" % repr(scan_angle))

        if not is_valid_scan_resolution(scan_resolution):
            raise SickConfigException("Undefined scan resolution: %s" % repr(scan_resolution))

        if self.operating_status.scan_angle != scan_angle or self.operating_status.scan_resolution != scan_resolution:
            self.logger.debug("Setting variant. Angle: %s, resolution: %s" % (scan_angle, scan_resolution))
            payload = b'\x3b'
            payload += sick_lms_scan_angles[scan_angle]
            payload += sick_lms_scan_resolutions[scan_resolution]

            self.send_message.payload = payload

            # This is done since the Sick stops sending data if the variant is reset midstream.
            self._set_opmode_monitor_request()

            response = self._send_message_and_get_reply(self.send_message.make_buffer())
            if response.payload[1] != 0x01:
                raise SickConfigException("Configuration was unsuccessful!")

            self.operating_status.scan_angle = response.parse_int(2, 2)
            self.operating_status.scan_resolution = response.parse_int(4, 2)

            self._make_angles()

            self._switch_opmode(self.session_mode, self.session_mode_params)

    def set_measuring_units(self, units: int = SICK_MEASURING_UNITS_MM):
        if is_valid_measuring_units(units) and units != self.config.measuring_units:
            self.logger.debug("Setting units: %s" % sick_lms_measuring_units[units])
            self.config.measuring_units = units
            self.config_has_updated = True

    def set_sensitivity(self, sensitivity=SICK_SENSITIVITY_STANDARD):
        if not is_valid_sensitivity(sensitivity):
            raise SickConfigException("Invalid sensitivity input: %s" % repr(sensitivity))

        if sensitivity != self.config.peak_threshold:
            self.logger.debug("Setting sensitivity: %s -> %s" % (self.config.peak_threshold, sensitivity))
            self.config.peak_threshold = sensitivity
            self.config_has_updated = True

    def set_measuring_mode(self, measuring_mode=SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE):
        if not is_valid_measuring_mode(measuring_mode):
            raise SickConfigException("Undefined measuring mode: '%s'" % repr(measuring_mode))

        if measuring_mode != self.config.measuring_mode:
            self.logger.debug("Setting measuring mode: %s -> %s" % (self.config.measuring_mode, measuring_mode))
            self.config.measuring_mode = measuring_mode
            self.config_has_updated = True

    def set_availability(self, availability_flag=SICK_FLAG_AVAILABILITY_DEFAULT):
        if availability_flag > 7:
            raise SickConfigException("Invalid availability: %s" % repr(availability_flag))

        if availability_flag != self.config.availability_level:
            self.logger.debug("Setting availability: %s -> %s" % (self.config.availability_level, availability_flag))
            # Maintain the higher level bits
            self.config.availability_level &= 0xf8

            # Set the new availability flags
            self.config.availability_level |= availability_flag

            self.config_has_updated = True

    # ----- data getters -----

    def get_scan(self, reflect_values=False):
        self._set_opmode_monitor_stream()
        response = self._recv_message()
        if reflect_values:
            if response.payload[0] != 0xc4:
                raise SickIOException("Invalid response for current measurement mode: %s" % response)
            self.c4_scan.parse_scan_profile(response.payload[1:], self.operating_status.measuring_mode)
            self.current_scan = self.c4_scan
        else:
            if response.payload[0] != 0xb0:
                raise SickIOException("Invalid response for current measurement mode: %s" % response)
            self.b0_scan.parse_scan_profile(response.payload[1:], self.operating_status.measuring_mode)
            self.current_scan = self.b0_scan

        self._make_distances()

    def get_scan_subrange(self, start_index, stop_index):
        self._set_opmode_monitor_stream_subrange(start_index, stop_index)
        response = self._recv_message()
        if response.payload[0] != 0xb7:
            raise SickIOException("Invalid response for current measurement mode: %s" % response)

        self.b7_scan.parse_scan_profile(response.payload[1:], self.operating_status.measuring_mode)
        self.current_scan = self.b7_scan

        self._make_distances()

    def get_partial_scan(self):
        self._set_opmode_monitor_stream_partial_scan()
        response = self._recv_message()
        if response.payload[0] != 0xb0:
            raise SickIOException("Invalid response for current measurement mode: %s" % response)

        self.b0_scan.parse_scan_profile(response.payload[1:], self.config.measuring_mode)
        self.current_scan = self.b0_scan

        self._make_distances()

    def get_mean_values(self, sample_size):
        self._set_opmode_monitor_stream_mean(sample_size)
        response = self._recv_message()
        if response.payload[0] != 0xb6:
            raise SickIOException("Invalid response for current measurement mode: %s" % response)

        self.b6_scan.parse_scan_profile(response.payload[1:], self.config.measuring_mode)
        self.current_scan = self.b6_scan

        self._make_distances()

    def get_mean_values_subrange(self, sample_size, start_index, stop_index):
        self._set_opmode_monitor_stream_mean_subrange(sample_size, start_index, stop_index)
        response = self._recv_message()
        if response.payload[0] != 0xb7:
            raise SickIOException("Invalid response for current measurement mode: %s" % response)

        self.b7_scan.parse_scan_profile(response.payload[1:], self.config.measuring_mode)
        self.current_scan = self.b7_scan

        self._make_distances()

    # ----- internal getters -----
    def _get_errors(self):
        payload = b'\x32'
        self.send_message.payload = payload

        self.logger.debug("Getting errors")
        response = self._send_message_and_get_reply(self.send_message.make_buffer())

        num_errors = int((len(response.payload) - 2) / 2)

        k = 1
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
        self.send_message.payload = b'\x31'
        self.logger.debug("Getting status")

        response = self._send_message_and_get_reply(self.send_message.make_buffer())

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
        self.send_message.payload = b'\x74'
        self.logger.debug("Getting configuration")
        response = self._send_message_and_get_reply(self.send_message.make_buffer())
        self.config.parse_config_profile(response)

    def _get_type(self):
        self.send_message.payload = b'\x3a'
        response = self._send_message_and_get_reply(self.send_message.make_buffer())

        model_string = response.parse_string(1).split(";")

        status = 0
        for key, value in supported_models.items():
            if type(key) == str:
                model = key.split(";")
                if model_string[0] == model[0] and model_string[1] == model[1]:
                    self.sick_type = value
                    status = 0
                    break
                elif model_string[0] == model[0] and model_string[1] != model[1]:
                    status = 1
                else:
                    status = 2

                self.sick_type = SICK_LMS_TYPE_UNKNOWN

        if status == 1:
            self.logger.debug("Device types match but software version is not supported")
        elif status == 2:
            self.logger.warning("LMS is of an unknown type: %s" % model_string)
        else:
            self.logger.debug("LMS is of type: %s" % model_string)

    # ----- user exposed lms functionality -----
    def update_config(self):
        if self.config_has_updated:
            self._set_opmode_installation()

            telegram = self.config.build_message()
            self.logger.debug("Setting new configuration")
            response = self._send_message_and_get_reply(telegram.make_buffer())
            if response.payload[1] != 0x01:
                raise SickConfigException("Configuration failed!")
            time.sleep(0.25)
            self.logger.debug("Configuration set! Switching back to mode: %s" %
                              sick_lms_operating_modes[self.session_mode])
            self._get_status()

            self._switch_opmode(self.session_mode, self.session_mode_params)

            self.config_has_updated = False
        else:
            self.logger.debug("Current configuration unchanged!")

    def reset(self):
        self.send_message.payload = b'\x10'
        self._timeout = 60
        self.logger.debug("Resetting...")
        self._send_message_and_get_reply(self.send_message.make_buffer(), reply_code=0x91)
        self._set_terminal_baud(DEFAULT_SICK_LMS_SICK_BAUD)

        self.logger.debug("Back online!")

        self._timeout = 30
        response = self._recv_message()

        if response.code != 0x90:
            self.logger.warning("Unexpected reply! (assuming device has been reset)")

        self.logger.debug("Starting back up")

        self._startup_lms()

    # ----- internal setters -----
    def _set_opmode_installation(self):
        self._switch_opmode(SICK_OP_MODE_INSTALLATION, DEFAULT_SICK_LMS_SICK_PASSWORD)
        self._timeout = DEFAULT_SICK_LMS_SICK_CONFIG_MESSAGE_TIMEOUT
        self.logger.debug("Successfully entered installation mode")

    def _set_opmode_diagnostic(self):
        self._switch_opmode(SICK_OP_MODE_DIAGNOSTIC)

    def _set_opmode_monitor_request(self):
        self._switch_opmode(SICK_OP_MODE_MONITOR_REQUEST_VALUES)

    def _set_opmode_monitor_stream(self):
        self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_VALUES)

    def _set_opmode_monitor_stream_range_reflectivity(self):
        self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT, [0x01, 0x00, 0xB5, 0x00])

    def _set_opmode_monitor_stream_partial_scan(self):
        self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN)

    def _set_opmode_monitor_stream_mean(self, sample_size):
        if not (2 < sample_size < 250):
            raise SickConfigException("Invalid sample size: %s" % repr(sample_size))
        if self.mean_sample_size != sample_size:
            self.mean_sample_size = sample_size

            mode_params = Message.int_to_byte(sample_size, 1)

            self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN, mode_params)
            self._timeout = DEFAULT_SICK_LMS_SICK_MEAN_VALUES_MESSAGE_TIMEOUT

    def _set_opmode_monitor_stream_subrange(self, start_index, stop_index):
        if self.subrange_start_index != start_index or self.subrange_stop_index != stop_index:
            max_subrange_stop_index = self.operating_status.scan_angle * 100 / self.operating_status.scan_resolution + 1
            if start_index > stop_index or start_index == 0 or stop_index > max_subrange_stop_index:
                raise SickConfigException("Invalid subregion bounds")

            self.subrange_start_index = start_index
            self.subrange_stop_index = stop_index

            mode_params = Message.int_to_byte(start_index, 2)
            mode_params += Message.int_to_byte(stop_index, 2)

            self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE, mode_params)

    def _set_opmode_monitor_stream_mean_subrange(self, sample_size, start_index, stop_index):
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

            self._switch_opmode(SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE, mode_params)
            self._timeout = DEFAULT_SICK_LMS_SICK_MEAN_VALUES_MESSAGE_TIMEOUT

    def _switch_opmode(self, mode: int, mode_params=None):
        if self.operating_status.operating_mode == mode:
            return

        self._timeout = DEFAULT_SICK_LMS_SICK_SWITCH_MODE_TIMEOUT

        payload_buffer = b'\x20'
        payload_buffer += Message.int_to_byte(mode, 1)

        if mode_params is not None:
            payload_buffer += mode_params
        message = Message(payload_buffer)

        self.logger.debug("Swiching to operating mode: %s" % sick_lms_operating_modes[mode])
        response = self._send_message_and_get_reply(message.make_buffer())
        if len(response.payload) < 2 or response.payload[1] != 0x00:
            raise SickConfigException("configuration request failed! Here's the response: %s" % response)

        self.operating_status.operating_mode = mode

        self._timeout = DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT

    def _set_terminal_baud(self, baud):
        self.serial_ref.baudrate = baud

    # ----- telegram transfer and protocol methods -----

    def _send_message_and_get_reply(self, message: bytes,
                                    num_tries=DEFAULT_SICK_LMS_NUM_TRIES, reply_code=None) -> Message:
        self.logger.debug("writing: %s" % message)
        self.serial_ref.write(message)

        if not self._check_for_ack():
            raise SickConfigException("Command not received correctly!")

        response = None
        for attempt in range(num_tries):
            try:
                response = self._recv_message()
                break
            except SickTimeoutException:
                self.logger.debug("Recevied timed out. Attempt %s of %s" % (attempt + 1, num_tries))
        if response is None:
            raise SickTimeoutException("Failed to get reply after %s attempts" % num_tries)

        if reply_code is not None:
            if response.code != reply_code:
                raise SickConfigException("Reply code doesn't match!")

        return response

    def _recv_message(self) -> Message:
        self.recv_message.reset()

        if self._read_8() == 0x02:
            if self._read_8() == 0x80:
                length = self._read_16()
                payload = self._read(length)
                checksum = self._read_16()

                self.recv_message.make_message(payload, checksum)

        # self.logger.debug("received: %s" % self.recv_message.buffer)
        return self.recv_message

    def _check_for_ack(self):
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
                elif ack == b'\x15' and attempts > DEFAULT_SICK_LMS_NUM_TRIES:
                    return False

                if time.time() - t0 > self._timeout:
                    attempts += 1
                    t0 = time.time()
                    self.logger.debug("ack check timed out. Attempt %s of %s" % (attempts, DEFAULT_SICK_LMS_NUM_TRIES))

        try:
            raise SickIOException("Invalid value received (neither ACK, nor NACK): %s" % responses)
        except SickIOException as error:
            self.logger.exception(error)
            raise

    # ----- pyserial interfaces -----

    def _read_with_timeout(self, n):
        t0 = time.time()
        response = b''
        while time.time() - t0 < self._timeout:
            response += self.serial_ref.read(n)
            if len(response) == n:
                break
        if time.time() - t0 > self._timeout:
            self.logger.error("Serial read timed out!")

        return response

    def _read(self, num_bytes) -> bytes:
        with self.serial_lock:
            result = b''
            if self._is_open():
                result = self._read_with_timeout(num_bytes)
            else:
                raise SickIOException("Serial not open for reading!")

        if len(result) == 0:
            raise SickTimeoutException("Failed to read serial!")

        self.recv_message.append_byte(result)
        return result

    def _read_8(self) -> int:
        return ord(self._read(1))

    def _read_16(self) -> int:
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
                 "Variant: %s\n" \
                 "Type: %s\n" \
                 "Sensor status: %s\n" \
                 "Scan angle: %sº\n" \
                 "Scan resolution: %sº\n" \
                 "Operating mode: %s\n" \
                 "Measuring mode: %s\n" \
                 "Measuring units: %s\n" % (
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
