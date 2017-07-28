from sicklms.message import Message
from sicklms.constants import *


class LmsOperatingStatus:
    def __init__(self):
        self.scan_angle = 0  # Sick scanning angle (deg)
        self.scan_resolution = 0  # Sick angular resolution (1/100 deg)
        self.num_motor_revs = 0  # Sick number of motor revs
        self.operating_mode = 0  # Sick operating mode
        self.measuring_mode = 0  # Sick measuring mode
        self.laser_mode = 0  # Sick laser is on/off
        self.device_status = 0  # Sick device status {ok,error}
        self.measuring_units = 0  # Sick measuring units {cm,mm}
        self.address = 0  # Sick device address
        self.variant = 0  # Sick variant {special,standard}

        self.system_software_version = [0] * 8  # Sick system software version
        self.prom_software_version = [0] * 8  # Sick boot prom software version

        self.restart_time = 0  # Sick restart time
        self.restart_mode = 0  # Sick restart mode

        self.pollution_vals = [0] * 8  # Calibrating the pollution channels
        self.pollution_calibration_vals = [0] * 8  # Calibrating the pollution channel values
        self.reference_pollution_vals = [0] * 4  # Reference pollution values
        self.reference_pollution_calibration_vals = [0] * 4  # Reference pollution calibration values

        # Receive signal amplitude in ADC incs when reference signal is switched off (Signal 1, Dark 100%)
        self.reference_scale_1_dark_100 = 0

        # Receive signal amplitude in ADC incs when reference signal is switched off (Signal 2, Dark 100%)
        self.reference_scale_2_dark_100 = 0

        # Receive signal amplitude in ADC incs when reference signal is switched off (Signal 1, Dark 66%)
        self.reference_scale_1_dark_66 = 0

        # Receive signal amplitude in ADC incs when reference signal is switched off (Signal 2, Dark 66%)
        self.reference_scale_2_dark_66 = 0

        self.signal_amplitude = 0  # Laser power in % of calibration value

        self.current_angle = 0  # Angle used for power measurement

        self.peak_threshold = 0  # Peak threshold in ADC incs for power measurement

        self.angle_of_measurement = 0  # Angles used to reference target for power measurement

        self.signal_amplitude_calibration_val = 0  # Calibration of the laser power

        self.stop_threshold_target_value = 0  # Target value of the stop threshold in ADC incs

        self.peak_threshold_target_value = 0  # Target value of the peak threshold in ADC incs

        self.stop_threshold_actual_value = 0  # Actual value of the stop threshold in ADC incs

        self.peak_threshold_actual_value = 0  # Actual value of the peak threshold in ADC incs

        # Reference target "single measured values." Low byte: Current number of filtered single measured values.
        # High byte: Max num filtered single measured value since power-on.
        self.reference_target_single_measured_vals = 0

        # Reference target "mean measured values." Low byte: Current number of filtered mean measured values.
        # High byte: Max num filtered mean measured value since power-on.
        self.reference_target_mean_measured_vals = 0

        self.field_evaluation_number = 0  # Number of evaluations when the field is infirnged (lies in [1,125])
        self.field_set_number = 0  # Active field set number
        # Offset for multiple evaluation of field set 2 (see page 105 of telegram listing)
        self.multiple_evaluation_offset_field_2 = 0

        self.baud_rate = 0  # Sick baud as reported by the device
        # 0 - When power is switched on baud rate is 9600/1 - configured transmission rate is used
        self.permanent_baud_rate = 0

    def parse_status(self, response: Message):
        self.operating_mode = response.payload[8]
        self.device_status = SICK_STATUS_ERROR if response.payload[8] else SICK_STATUS_OK
        self.num_motor_revs = response.parse_int(67, 2)
        self.measuring_mode = response.payload[102]
        self.scan_angle = response.parse_int(107, 2)
        self.scan_resolution = response.parse_int(109, 2)
        self.variant = response.payload[18]
        self.address = response.payload[120]
        self.measuring_units = response.payload[122]
        self.laser_mode = response.payload[123]
        self.system_software_version = response.parse_string(1, 7)
        self.prom_software_version = response.parse_string(124, 7)
        self.restart_mode = response.payload[111]
        self.restart_time = response.parse_int(112, 2)

        payload_index = 19
        for index in range(len(self.pollution_vals)):
            self.pollution_vals[index] = response.parse_int(payload_index, 2)
            payload_index += 2

        payload_index = 35
        for index in range(len(self.reference_pollution_vals)):
            self.reference_pollution_vals[index] = response.parse_int(payload_index, 2)
            payload_index += 2

        payload_index = 43
        for index in range(len(self.pollution_calibration_vals)):
            self.pollution_calibration_vals[index] = response.parse_int(payload_index, 2)
            payload_index += 2

        payload_index = 59
        for index in range(len(self.reference_pollution_calibration_vals)):
            self.reference_pollution_calibration_vals[index] = response.parse_int(payload_index, 2)
            payload_index += 2

        self.reference_scale_1_dark_100 = response.parse_int(71, 2)
        self.reference_scale_2_dark_100 = response.parse_int(75, 2)
        self.reference_scale_1_dark_66 = response.parse_int(77, 2)
        self.reference_scale_2_dark_66 = response.parse_int(81, 2)
        self.signal_amplitude = response.parse_int(83, 2)
        self.current_angle = response.parse_int(85, 2)
        self.peak_threshold = response.parse_int(87, 2)
        self.angle_of_measurement = response.parse_int(89, 2)
        self.signal_amplitude_calibration_val = response.parse_int(91, 2)
        self.stop_threshold_target_value = response.parse_int(93, 2)
        self.peak_threshold_target_value = response.parse_int(95, 2)
        self.stop_threshold_actual_value = response.parse_int(97, 2)
        self.peak_threshold_actual_value = response.parse_int(99, 2)
        self.reference_target_single_measured_vals = response.parse_int(103, 2)
        self.reference_target_mean_measured_vals = response.parse_int(105, 2)
        self.multiple_evaluation_offset_field_2 = response.payload[114]
        self.field_evaluation_number = response.payload[118]
        self.field_set_number = response.payload[121]
        self.permanent_baud_rate = response.payload[119]
        self.baud_rate = response.parse_int(116, 2)


class LmsConfig:
    def __init__(self):
        self.measuring_mode = 0  # Sick measuring mode

        self.measuring_units = 0  # Sick measuring units {cm,mm}

        # Maximum diameter of objects that are not to be detected (units cm)
        self.blanking = 0

        self.fields_b_c_restart_times = 0  # Restart times for fields B and C

        # Number of scans that take place before LMS switches the outputs (only applies to availability level 1)
        self.dazzling_multiple_evaluation = 0

        # Peak threshold/black correction (This applies to Sick LMS 200/220 models,
        # when Sick LMS 211/221/291 models are used, this value is sensitivity)
        self.peak_threshold = 0

        self.stop_threshold = 0  # # Stop threshold in mV (This only applies to Sick LMS 200/220 models)

        self.availability_level = 0  # Availability level of the Sick LMS

        self.temporary_field = 0  # Indicates whether fields A and B are subtractive

        self.subtractive_fields = 0  # Indicates whether fields A and B are subtractive

        self.multiple_evaluation = 0  # Multiple evalutation of scan data

        self.sick_restart = 0  # Indicates the restart level of the device

        self.restart_time = 0  # Inidicates the restart time of the device

        # Multiple evaluation for objects less than the blanking size
        self.multiple_evaluation_suppressed_objects = 0

        self.contour_a_reference = 0  # Contour function A

        # When contour function is active the positive tolerance is defined in (cm)
        self.contour_a_positive_tolerance_band = 0

        # When contour function is active the negative tolerance is defined in (cm)
        self.contour_a_negative_tolerance_band = 0

        # When contour function is active the start angle of area to be monitored is defined (deg)
        self.contour_a_start_angle = 0

        # When contour function is active the stop angle of area to be monitored is defined (deg)
        self.contour_a_stop_angle = 0

        self.contour_b_reference = 0  # Contour function B

        # When contour function is active the positive tolerance is defined in (cm)
        self.contour_b_positive_tolerance_band = 0

        # When contour function is active the negative tolerance is defined in (cm)
        self.contour_b_negative_tolerance_band = 0

        # When contour function is active the start angle of area to be monitored is defined (deg)
        self.contour_b_start_angle = 0

        # When contour function is active the stop angle of area to be monitored is defined (deg)
        self.contour_b_stop_angle = 0

        self.contour_c_reference = 0  # Contour function C

        # When contour function is active the positive tolerance is defined in (cm)
        self.contour_c_positive_tolerance_band = 0

        # When contour function is active the negative tolerance is defined in (cm)
        self.contour_c_negative_tolerance_band = 0

        # When contour function is active the start angle of area to be monitored is defined (deg)
        self.contour_c_start_angle = 0

        # When contour function is active the stop angle of area to be monitored is defined (deg)
        self.contour_c_stop_angle = 0

        self.pixel_oriented_evaluation = 0  # Pixel oriented evaluation

        self.single_measured_value_evaluation_mode = 0  # Multiple evaluation (min: 1, max: 125)

    def parse_config_profile(self, response: Message):
        self.blanking = response.parse_int(0, 2)
        self.stop_threshold = response.payload[2]
        self.peak_threshold = response.payload[3]

        self.availability_level = response.payload[4]
        self.measuring_mode = response.payload[5]
        self.measuring_units = response.payload[6]

        self.temporary_field = response.payload[7]
        self.subtractive_fields = response.payload[8]
        self.multiple_evaluation = response.payload[9]
        self.sick_restart = response.payload[10]
        self.restart_time = response.payload[11]
        self.multiple_evaluation_suppressed_objects = response.payload[12]

        self.contour_a_reference = response.payload[13]
        self.contour_a_positive_tolerance_band = response.payload[14]
        self.contour_a_negative_tolerance_band = response.payload[15]
        self.contour_a_start_angle = response.payload[16]
        self.contour_a_stop_angle = response.payload[17]

        self.contour_b_reference = response.payload[18]
        self.contour_b_positive_tolerance_band = response.payload[19]
        self.contour_b_negative_tolerance_band = response.payload[20]
        self.contour_b_start_angle = response.payload[21]
        self.contour_b_stop_angle = response.payload[22]

        self.contour_c_reference = response.payload[23]
        self.contour_c_positive_tolerance_band = response.payload[24]
        self.contour_c_negative_tolerance_band = response.payload[25]
        self.contour_c_start_angle = response.payload[26]
        self.contour_c_stop_angle = response.payload[27]

        self.pixel_oriented_evaluation = response.payload[28]
        self.single_measured_value_evaluation_mode = response.payload[29]

        self.fields_b_c_restart_times = response.parse_int(30, 2)
        self.dazzling_multiple_evaluation = response.parse_int(32, 2)

    def build_message(self) -> Message:
        payload = b'\x77\x00\x00\x70\x00\x00\x00\x01\x00\x00\x02\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00' \
                  b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00'

        payload = payload[0:6] + \
                  Message.int_to_byte(self.measuring_mode, 1) + \
                  Message.int_to_byte(self.measuring_units, 1) + \
                  payload[8:]

        # payload = b'\x77'
        # payload += Message.int_to_byte(self.blanking, 2)
        # if self.stop_threshold not in sick_lms_peak_thresholds:
        #     raise SickConfigException("Invalid peak threshold: %s" % self.stop_threshold)
        # payload += Message.int_to_byte(self.stop_threshold, 1)
        #
        # payload += Message.int_to_byte(self.peak_threshold, 1)
        #
        # if self.availability_level not in sick_lms_flag_availabilities:
        #     raise SickConfigException("Invalid availability: %s" % self.availability_level)
        # payload += Message.int_to_byte(self.availability_level, 1)
        #
        # if self.measuring_mode not in sick_lms_measuring_modes:
        #     raise SickConfigException("Invalid measuring mode: %s" % self.measuring_mode)
        # payload += Message.int_to_byte(self.measuring_mode, 1)
        #
        # if self.measuring_units not in sick_lms_measuring_units:
        #     raise SickConfigException("Invalid measuring mode: %s" % self.measuring_units)
        # payload += Message.int_to_byte(self.measuring_units, 1)
        #
        # payload += Message.int_to_byte(self.temporary_field, 1)
        # payload += Message.int_to_byte(self.subtractive_fields, 1)
        #
        # payload += Message.int_to_byte(self.multiple_evaluation, 1)
        # payload += Message.int_to_byte(self.sick_restart, 1)
        # payload += Message.int_to_byte(self.restart_time, 1)
        # payload += Message.int_to_byte(self.multiple_evaluation_suppressed_objects, 1)
        #
        # payload += Message.int_to_byte(self.contour_a_reference, 1)
        # payload += Message.int_to_byte(self.contour_a_positive_tolerance_band, 1)
        # payload += Message.int_to_byte(self.contour_a_negative_tolerance_band, 1)
        # payload += Message.int_to_byte(self.contour_a_start_angle, 1)
        # payload += Message.int_to_byte(self.contour_a_stop_angle, 1)
        #
        # payload += Message.int_to_byte(self.contour_b_reference, 1)
        # payload += Message.int_to_byte(self.contour_b_positive_tolerance_band, 1)
        # payload += Message.int_to_byte(self.contour_b_negative_tolerance_band, 1)
        # payload += Message.int_to_byte(self.contour_b_start_angle, 1)
        # payload += Message.int_to_byte(self.contour_b_stop_angle, 1)
        #
        # payload += Message.int_to_byte(self.contour_c_reference, 1)
        # payload += Message.int_to_byte(self.contour_c_positive_tolerance_band, 1)
        # payload += Message.int_to_byte(self.contour_c_negative_tolerance_band, 1)
        # payload += Message.int_to_byte(self.contour_c_start_angle, 1)
        # payload += Message.int_to_byte(self.contour_c_stop_angle, 1)
        #
        # payload += Message.int_to_byte(self.pixel_oriented_evaluation, 1)
        # payload += Message.int_to_byte(self.single_measured_value_evaluation_mode, 1)
        # payload += Message.int_to_byte(self.fields_b_c_restart_times, 2)
        # payload += Message.int_to_byte(self.dazzling_multiple_evaluation, 2)

        return Message(payload)


class ScanProfile:
    def __init__(self):
        # Number of measurements
        self.num_measurements = 0

        # Range/reflectivity measurement buffer
        self.measurements = [0] * SICK_MAX_NUM_MEASUREMENTS

        # Reflects the Field A bit value returned w/ range measurement
        self.field_a_values = [0] * SICK_MAX_NUM_MEASUREMENTS

        # Reflects the Field B but value returned w/ range measurement
        self.field_b_values = [0] * SICK_MAX_NUM_MEASUREMENTS

        # Reflects the Field C (or dazzle - depending upon sensor mode) value returned w/ range measurement
        self.field_c_values = [0] * SICK_MAX_NUM_MEASUREMENTS

        # Telegram index modulo 256
        self.telegram_index = 0

        # If real-time scan indices are requested, this value is set (modulo 256)
        self.real_time_scan_index = 0

        self.availability_level = 0

    def _extract_measurement_values(self, byte_sequence, measuring_mode):
        if measuring_mode == SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE:
            for index in range(self.num_measurements):
                self.measurements[index] = byte_sequence[index * 2] + 256 * (byte_sequence[index * 2 + 1] & 0x1f)
                self.field_a_values[index] = byte_sequence[index * 2 + 1] & 0x20
                self.field_b_values[index] = byte_sequence[index * 2 + 1] & 0x40
                self.field_c_values[index] = byte_sequence[index * 2 + 1] & 0x80

        elif measuring_mode == SICK_MS_MODE_8_OR_80_REFLECTOR:
            for index in range(self.num_measurements):
                self.measurements[index] = byte_sequence[index * 2] + 256 * (byte_sequence[index * 2 + 1] & 0x1f)
                self.field_a_values[index] = byte_sequence[index * 2 + 1] & 0xe0

        elif measuring_mode == SICK_MS_MODE_8_OR_80_FA_FB_FC:
            for index in range(self.num_measurements):
                self.measurements[index] = byte_sequence[index * 2] + 256 * (byte_sequence[index * 2 + 1] & 0x1f)

                self.field_a_values[index] = byte_sequence[index * 2 + 1] & 0x20
                self.field_b_values[index] = byte_sequence[index * 2 + 1] & 0x40
                self.field_c_values[index] = byte_sequence[index * 2 + 1] & 0x80

        elif measuring_mode == SICK_MS_MODE_16_REFLECTOR:
            for index in range(self.num_measurements):
                self.measurements[index] = byte_sequence[index * 2] + 256 * (byte_sequence[index * 2 + 1] & 0x3f)
                self.field_a_values[index] = byte_sequence[index * 2 + 1] & 0xc0

        elif measuring_mode == SICK_MS_MODE_16_FA_FB:
            for index in range(self.num_measurements):
                self.measurements[index] = byte_sequence[index * 2] + 256 * (byte_sequence[index * 2 + 1] & 0x3f)

                self.field_a_values[index] = byte_sequence[index * 2 + 1] & 0x40
                self.field_b_values[index] = byte_sequence[index * 2 + 1] & 0x80

        elif measuring_mode == SICK_MS_MODE_32_REFLECTOR or measuring_mode == SICK_MS_MODE_32_FA:
            for index in range(self.num_measurements):
                self.measurements[index] = byte_sequence[index * 2] + 256 * (byte_sequence[index * 2 + 1] & 0x7f)
                self.field_a_values[index] = byte_sequence[index * 2 + 1] & 0x80

        elif measuring_mode == SICK_MS_MODE_32_IMMEDIATE:
            for index in range(self.num_measurements):
                self.measurements[index] = byte_sequence[index * 2] + (byte_sequence[index * 2 + 1] << 8)

        elif measuring_mode == SICK_MS_MODE_REFLECTIVITY:
            for index in range(self.num_measurements):
                self.measurements[index] = byte_sequence[index * 2] + (byte_sequence[index * 2 + 1] << 8)

    def parse_scan_profile(self, payload: bytes, measuring_mode):
        pass

    def _is_returning_real_time_indicies(self):
        return bool(self.availability_level & SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES)


class ScanProfileB0(ScanProfile):
    def __init__(self):
        # Indicates the start angle of the scan (This is useful for partial scans)
        self.partial_scan_index = 0

        super(ScanProfileB0, self).__init__()

    def parse_scan_profile(self, payload: bytes, measuring_mode):
        self.num_measurements = payload[0] + 256 * (payload[1] & 0x03)
        self.partial_scan_index = (payload[1] & 0x18) >> 3

        self._extract_measurement_values(payload[2:], measuring_mode)
        data_offset = 2 + 2 * self.num_measurements

        if self._is_returning_real_time_indicies():
            self.real_time_scan_index = payload[data_offset]
            data_offset += 1

        self.telegram_index = payload[data_offset]


class ScanProfileB6(ScanProfile):
    def __init__(self):
        self.sample_size = 0  # Number of scans used in computing the returned mean

        super(ScanProfileB6, self).__init__()

    def parse_scan_profile(self, payload: bytes, measuring_mode):
        self.sample_size = payload[0]
        self.num_measurements = payload[1] + 256 * (payload[2] & 0x03)

        self._extract_measurement_values(payload[3:], measuring_mode)

        data_offset = 3 + 2 * self.num_measurements
        if self._is_returning_real_time_indicies():
            self.real_time_scan_index = payload[data_offset]
            data_offset += 1

        self.telegram_index = payload[data_offset]


class ScanProfileB7(ScanProfile):
    def __init__(self):
        # Measurement subrange start index
        self.subrange_start_index = 0

        # Measurement subrange stop index
        self.subrange_stop_index = 0

        # Indicates the start angle of the scan (This is useful for partial scans)
        self.partial_scan_index = 0

        super(ScanProfileB7, self).__init__()

    def parse_scan_profile(self, payload: bytes, measuring_mode):
        self.subrange_start_index = payload[0] + (payload[1] << 8)
        self.subrange_stop_index = payload[2] + (payload[3] << 8)
        self.num_measurements = payload[4] + 256 * (payload[5] & 0x03)
        self.partial_scan_index = (payload[5] & 0x18) >> 3

        self._extract_measurement_values(payload[6:], measuring_mode)

        data_offset = 6 + 2 * self.num_measurements
        if self._is_returning_real_time_indicies():
            self.real_time_scan_index = payload[data_offset]
            data_offset += 1
        self.telegram_index = payload[data_offset]


class ScanProfileBF(ScanProfile):
    def __init__(self):
        # Measurement subrange start index
        self.subrange_start_index = 0

        # Measurement subrange stop index
        self.subrange_stop_index = 0

        super(ScanProfileBF, self).__init__()

    def parse_scan_profile(self, payload: bytes, measuring_mode):
        self.sample_size = payload[0]
        self.subrange_start_index = payload[1] + (payload[2] << 8)
        self.subrange_stop_index = payload[3] + (payload[4] << 8)
        self.num_measurements = payload[5] + 256 * (payload[6] & 0x03)

        self._extract_measurement_values(payload[7:], measuring_mode)

        data_offset = 7 + 2 * self.num_measurements
        if self._is_returning_real_time_indicies():
            self.real_time_scan_index = payload[data_offset]
            data_offset += 1
        self.telegram_index = payload[data_offset]


class ScanProfileC4(ScanProfile):
    def __init__(self):
        # Number of range measurements
        self.num_range_measurements = 0

        # Number of reflectivity measurements
        self.num_reflect_measurements = 0

        # Range measurement buffer
        self.range_measurements = [0] * SICK_MAX_NUM_MEASUREMENTS

        # Reflect measurements buffer
        self.reflect_measurements = [0] * SICK_MAX_NUM_MEASUREMENTS

        # Start index of the measured reflectivity value subrange
        self.reflect_subrange_start_index = 0

        # Stop index of the measured reflectivity value subrange
        self.reflect_subrange_stop_index = 0

        super(ScanProfileC4, self).__init__()

    def parse_scan_profile(self, payload: bytes, measuring_mode):
        self.num_measurements = payload[0] + 256 * (payload[1] & 0x03)

        self._extract_measurement_values(payload[2:], measuring_mode)

        data_offset = 2 + 2 * self.num_measurements
        self.num_reflect_measurements = payload[data_offset] + 256 * (payload[data_offset + 1] & 0x03)
        data_offset += 2

        self.reflect_subrange_start_index = payload[data_offset] + (payload[data_offset + 1] << 8)
        data_offset += 2

        self.reflect_subrange_stop_index = payload[data_offset] + (payload[data_offset + 1] << 8)
        data_offset += 2

        for index in range(self.num_reflect_measurements):
            self.reflect_measurements[index] = payload[data_offset]
            data_offset += 1

        if self._is_returning_real_time_indicies():
            self.real_time_scan_index = payload[data_offset]
            data_offset += 1
        self.telegram_index = payload[data_offset]
