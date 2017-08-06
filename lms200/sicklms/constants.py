def make_reversible(d):
    d.update({v: k for k, v in d.items()})


DEFAULT_SICK_LMS_SICK_BAUD = 9600
DEFAULT_SICK_LMS_HOST_ADDRESS = 0x80
DEFAULT_SICK_LMS_SICK_ADDRESS = 0x00
DEFAULT_SICK_LMS_SICK_PASSWORD = b"SICK_LMS"
DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT = 1.5
DEFAULT_SICK_LMS_SICK_SWITCH_MODE_TIMEOUT = 3
DEFAULT_SICK_LMS_SICK_MEAN_VALUES_MESSAGE_TIMEOUT = 15
DEFAULT_SICK_LMS_SICK_CONFIG_MESSAGE_TIMEOUT = 15
DEFAULT_SICK_LMS_BYTE_INTERVAL = 0.000055
DEFAULT_SICK_LMS_NUM_TRIES = 3

SICK_MAX_NUM_MEASUREMENTS = 721

definitions = {
    DEFAULT_SICK_LMS_SICK_BAUD                       : "DEFAULT_SICK_LMS_SICK_BAUD",
    DEFAULT_SICK_LMS_HOST_ADDRESS                    : "DEFAULT_SICK_LMS_HOST_ADDRESS",
    DEFAULT_SICK_LMS_SICK_ADDRESS                    : "DEFAULT_SICK_LMS_SICK_ADDRESS",
    DEFAULT_SICK_LMS_SICK_PASSWORD                   : "DEFAULT_SICK_LMS_SICK_PASSWORD",
    DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT            : "DEFAULT_SICK_LMS_SICK_MESSAGE_TIMEOUT",
    DEFAULT_SICK_LMS_SICK_SWITCH_MODE_TIMEOUT        : "DEFAULT_SICK_LMS_SICK_SWITCH_MODE_TIMEOUT",
    DEFAULT_SICK_LMS_SICK_MEAN_VALUES_MESSAGE_TIMEOUT: "DEFAULT_SICK_LMS_SICK_MEAN_VALUES_MESSAGE_TIMEOUT",
    DEFAULT_SICK_LMS_SICK_CONFIG_MESSAGE_TIMEOUT     : "DEFAULT_SICK_LMS_SICK_CONFIG_MESSAGE_TIMEOUT",
    DEFAULT_SICK_LMS_BYTE_INTERVAL                   : "DEFAULT_SICK_LMS_BYTE_INTERVAL",
    DEFAULT_SICK_LMS_NUM_TRIES                       : "DEFAULT_SICK_LMS_NUM_TRIES",
}

SICK_LMS_TYPE_200_30106 = 0  # Sick LMS type 200-30106

# Supported 211 models
SICK_LMS_TYPE_211_30106 = 1  # Sick LMS type 211-30106
SICK_LMS_TYPE_211_30206 = 2  # Sick LMS type 211-30206
SICK_LMS_TYPE_211_S07 = 3  # Sick LMS type 211-S07
SICK_LMS_TYPE_211_S14 = 4  # Sick LMS type 211-S14
SICK_LMS_TYPE_211_S15 = 5  # Sick LMS type 211-S15
SICK_LMS_TYPE_211_S19 = 6  # Sick LMS type 211-S19
SICK_LMS_TYPE_211_S20 = 7  # Sick LMS type 211-S20

# Supported 220 models
SICK_LMS_TYPE_220_30106 = 8  # Sick LMS type 220-30106

# Supported 221 models
SICK_LMS_TYPE_221_30106 = 9  # Sick LMS type 221-30106
SICK_LMS_TYPE_221_30206 = 10  # Sick LMS type 221-30206
SICK_LMS_TYPE_221_S07 = 11  # Sick LMS type 221-S07
SICK_LMS_TYPE_221_S14 = 12  # Sick LMS type 221-S14
SICK_LMS_TYPE_221_S15 = 13  # Sick LMS type 221-S15
SICK_LMS_TYPE_221_S16 = 14  # Sick LMS type 221-S16
SICK_LMS_TYPE_221_S19 = 15  # Sick LMS type 221-S19
SICK_LMS_TYPE_221_S20 = 16  # Sick LMS type 221-S20

# Supported 291 models
SICK_LMS_TYPE_291_S05 = 17  # Sick LMS type 291-S05
SICK_LMS_TYPE_291_S14 = 18  # Sick LMS type 291-S14 (LMS Fast)
SICK_LMS_TYPE_291_S15 = 19
SICK_LMS_TYPE_200_S18063 = 20
SICK_LMS_TYPE_UNKNOWN = 0xff

supported_models = {
    SICK_LMS_TYPE_200_S18063: "LMS200;S18063",
    SICK_LMS_TYPE_200_30106: "LMS200;30106",
    SICK_LMS_TYPE_211_30106: "LMS211;30106",
    SICK_LMS_TYPE_211_30206: "LMS211;30206",
    SICK_LMS_TYPE_211_S07  : "LMS211;S07",
    SICK_LMS_TYPE_211_S14  : "LMS211;S14",
    SICK_LMS_TYPE_211_S15  : "LMS211;S15",
    SICK_LMS_TYPE_211_S19  : "LMS211;S19",
    SICK_LMS_TYPE_211_S20  : "LMS211;S20",
    SICK_LMS_TYPE_221_30106: "LMS220;30106",
    SICK_LMS_TYPE_221_30206: "LMS221;30106",
    SICK_LMS_TYPE_221_S07  : "LMS221;30206",
    SICK_LMS_TYPE_221_S14  : "LMS221;S07",
    SICK_LMS_TYPE_221_S15  : "LMS221;S14",
    SICK_LMS_TYPE_221_S16  : "LMS221;S15",
    SICK_LMS_TYPE_221_S19  : "LMS221;S16",
    SICK_LMS_TYPE_221_S20  : "LMS221;S19",
    SICK_LMS_TYPE_291_S05  : "LMS221;S20",
    SICK_LMS_TYPE_291_S14  : "LMS291;S05",
    SICK_LMS_TYPE_291_S15  : "LMS291;S14",
    SICK_LMS_TYPE_UNKNOWN  : "UNKNOWN",
}

make_reversible(supported_models)

SICK_LMS_VARIANT_2XX_TYPE_6 = 0x00  # Standard LMS 2xx type 6 models
SICK_LMS_VARIANT_SPECIAL = 0x01  # Special models (i.e. LMS211-/221-S19/-S20
SICK_LMS_VARIANT_UNKNOWN = 0xFF  # Unknown LMS variant

sick_lms_variants = {
    SICK_LMS_VARIANT_2XX_TYPE_6: "SICK_LMS_VARIANT_2XX_TYPE_6",
    SICK_LMS_VARIANT_SPECIAL   : "SICK_LMS_VARIANT_SPECIAL",
    SICK_LMS_VARIANT_UNKNOWN   : "SICK_LMS_VARIANT_UNKNOWN",
}

SICK_SCAN_ANGLE_90 = 90  # Scanning angle of 90 degrees
SICK_SCAN_ANGLE_100 = 100  # Scanning angle of 100 degrees
SICK_SCAN_ANGLE_180 = 180  # Scanning angle of 180 degrees
SICK_SCAN_ANGLE_UNKNOWN = 0xFF  # Unknown scanning angle

sick_lms_scan_angles = {
    SICK_SCAN_ANGLE_90     : b'\xff',
    SICK_SCAN_ANGLE_100    : b'\x64',
    SICK_SCAN_ANGLE_180    : b'\xb4',
    SICK_SCAN_ANGLE_UNKNOWN: b'\xff',
}

SICK_SCAN_RESOLUTION_25 = 25  # 0.25 degree angular resolution
SICK_SCAN_RESOLUTION_50 = 50  # 0.50 degree angular resolution
SICK_SCAN_RESOLUTION_100 = 100  # 1.00 degree angular resolution
SICK_SCAN_RESOLUTION_UNKNOWN = 0xFF  # Unknown angular resolution

sick_lms_scan_resolutions = {
    SICK_SCAN_RESOLUTION_25     : b'\x64',
    SICK_SCAN_RESOLUTION_50     : b'\x32',
    SICK_SCAN_RESOLUTION_100    : b'\x19',
    SICK_SCAN_RESOLUTION_UNKNOWN: b'\xff',
}

SICK_MEASURING_UNITS_CM = 0x00  # Measured values are in centimeters
SICK_MEASURING_UNITS_MM = 0x01  # Measured values are in milimeters
SICK_MEASURING_UNITS_UNKNOWN = 0xFF  # Unknown units

sick_lms_measuring_units = {
    SICK_MEASURING_UNITS_CM     : "SICK_MEASURING_UNITS_CM",
    SICK_MEASURING_UNITS_MM     : "SICK_MEASURING_UNITS_MM",
    SICK_MEASURING_UNITS_UNKNOWN: "SICK_MEASURING_UNITS_UNKNOWN",
}
make_reversible(sick_lms_measuring_units)

SICK_SENSITIVITY_STANDARD = 0x00  # Standard sensitivity: 30m @ 10% reflectivity
SICK_SENSITIVITY_MEDIUM = 0x01  # Medium sensitivity:   25m @ 10% reflectivity
SICK_SENSITIVITY_LOW = 0x02  # Low sensitivity:      20m @ 10% reflectivity
SICK_SENSITIVITY_HIGH = 0x03  # High sensitivity:     42m @ 10% reflectivity
SICK_SENSITIVITY_UNKNOWN = 0xFF  # Sensitivity unknown

sick_lms_sensitivities = {
    SICK_SENSITIVITY_STANDARD: "SICK_SENSITIVITY_STANDARD",
    SICK_SENSITIVITY_MEDIUM  : "SICK_SENSITIVITY_MEDIUM",
    SICK_SENSITIVITY_LOW     : "SICK_SENSITIVITY_LOW",
    SICK_SENSITIVITY_HIGH    : "SICK_SENSITIVITY_HIGH",
    SICK_SENSITIVITY_UNKNOWN : "SICK_SENSITIVITY_UNKNOWN",
}

SICK_PEAK_THRESHOLD_DETECTION_WITH_NO_BLACK_EXTENSION = 0x00  # Standard: peak threshold detection, no black extension
SICK_PEAK_THRESHOLD_DETECTION_WITH_BLACK_EXTENSION = 0x01  # Peak threshold detection, active black extension
SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_NO_BLACK_EXTENSION = 0x02  # No peak threshold detection, no black extension
SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_BLACK_EXTENSION = 0x03  # No peak threshold detection, active black extension
SICK_PEAK_THRESHOLD_UNKNOWN = 0xFF  # Peak threshold unknown

sick_lms_peak_thresholds = {
    SICK_PEAK_THRESHOLD_DETECTION_WITH_NO_BLACK_EXTENSION   : "SICK_PEAK_THRESHOLD_DETECTION_WITH_NO_BLACK_EXTENSION",
    SICK_PEAK_THRESHOLD_DETECTION_WITH_BLACK_EXTENSION      : "SICK_PEAK_THRESHOLD_DETECTION_WITH_BLACK_EXTENSION",
    SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_NO_BLACK_EXTENSION: "SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_NO_BLACK_EXTENSION",
    SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_BLACK_EXTENSION   : "SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_BLACK_EXTENSION",
    SICK_PEAK_THRESHOLD_UNKNOWN                             : "SICK_PEAK_THRESHOLD_UNKNOWN",
}

SICK_STATUS_OK = 0x00  # LMS is OK
SICK_STATUS_ERROR = 0x01  # LMS has encountered an error
SICK_STATUS_UNKNOWN = 0xFF  # Unknown LMS status

sick_lms_status = {
    SICK_STATUS_OK     : "SICK_STATUS_OK",
    SICK_STATUS_ERROR  : "SICK_STATUS_ERROR",
    SICK_STATUS_UNKNOWN: "SICK_STATUS_UNKNOWN",
}

SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE = 0x00  # Measurement range 8m/80m; fields A,B and Dazzle (Default)
SICK_MS_MODE_8_OR_80_REFLECTOR = 0x01  # Measurement range 8/80m; reflector bits in 8 levels
SICK_MS_MODE_8_OR_80_FA_FB_FC = 0x02  # Measurement range 8/80m; fields A,B, and C
SICK_MS_MODE_16_REFLECTOR = 0x03  # Measurement range 16m; reflector bits in 4 levels
SICK_MS_MODE_16_FA_FB = 0x04  # Measurement range 16m; fields A and B
SICK_MS_MODE_32_REFLECTOR = 0x05  # Measurement range 32m; reflector bit in 2 levels
SICK_MS_MODE_32_FA = 0x06  # Measurement range 32m; field A
SICK_MS_MODE_32_IMMEDIATE = 0x0F  # Measurement range 32m; immediate data transmission, no flags
SICK_MS_MODE_REFLECTIVITY = 0x3F  # Sick LMS 2xx returns reflectivity (echo amplitude) values instead of range measurements
SICK_MS_MODE_UNKNOWN = 0xFF  # Unknown range

sick_lms_measuring_modes = {
    SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE: "SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE",
    SICK_MS_MODE_8_OR_80_REFLECTOR   : "SICK_MS_MODE_8_OR_80_REFLECTOR",
    SICK_MS_MODE_8_OR_80_FA_FB_FC    : "SICK_MS_MODE_8_OR_80_FA_FB_FC",
    SICK_MS_MODE_16_REFLECTOR        : "SICK_MS_MODE_16_REFLECTOR",
    SICK_MS_MODE_16_FA_FB            : "SICK_MS_MODE_16_FA_FB",
    SICK_MS_MODE_32_REFLECTOR        : "SICK_MS_MODE_32_REFLECTOR",
    SICK_MS_MODE_32_FA               : "SICK_MS_MODE_32_FA",
    SICK_MS_MODE_32_IMMEDIATE        : "SICK_MS_MODE_32_IMMEDIATE",
    SICK_MS_MODE_REFLECTIVITY        : "SICK_MS_MODE_REFLECTIVITY",
    SICK_MS_MODE_UNKNOWN             : "SICK_MS_MODE_UNKNOWN",
}

SICK_OP_MODE_INSTALLATION = 0x00  # Installation mode for writing EEPROM
SICK_OP_MODE_DIAGNOSTIC = 0x10  # Diagnostic mode for testing purposes
SICK_OP_MODE_MONITOR_STREAM_MIN_VALUE_FOR_EACH_SEGMENT = 0x20  # Streams minimum measured values for each segement
SICK_OP_MODE_MONITOR_TRIGGER_MIN_VALUE_ON_OBJECT = 0x21  # Sends the min measured values when object is detected
SICK_OP_MODE_MONITOR_STREAM_MIN_VERT_DIST_TO_OBJECT = 0x22  # Streams min "vertical distance" to objects
SICK_OP_MODE_MONITOR_TRIGGER_MIN_VERT_DIST_TO_OBJECT = 0x23  # Sends min vertical distance to object when detected
SICK_OP_MODE_MONITOR_STREAM_VALUES = 0x24  # Streams all measured values in a scan
SICK_OP_MODE_MONITOR_REQUEST_VALUES = 0x25  # Sends measured range values on request (i.e. when polled)
SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES = 0x26  # Streams mean values from a sample size of n consecutive scans
SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE = 0x27  # Streams data from given subrange
SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE = 0x28  # Streams mean values over requested subrange
SICK_OP_MODE_MONITOR_STREAM_VALUES_WITH_FIELDS = 0x29  # Streams measured values with associated flags
SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN = 0x2A  # Streams measured values of partial scan directly after measurement
SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT_FROM_PARTIAL_SCAN = 0x2B  # Streams range and intensity from n partial scans
SICK_OP_MODE_MONITOR_STREAM_MIN_VALUES_FOR_EACH_SEGMENT_SUBRANGE = 0x2C  # Streams minimum measured values for each segment in a sub-range
SICK_OP_MODE_MONITOR_NAVIGATION = 0x2E  # Sick outputs navigation data records
SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT = 0x50  # Streams measured range from a scan and sub-range of reflectivity values
SICK_OP_MODE_UNKNOWN = 0xFF  # Unknown operating mode

sick_lms_operating_modes = {
    SICK_OP_MODE_INSTALLATION                                       : "INSTALLATION",
    SICK_OP_MODE_DIAGNOSTIC                                         : "DIAGNOSTIC",
    SICK_OP_MODE_MONITOR_STREAM_MIN_VALUE_FOR_EACH_SEGMENT          : "MONITOR_STREAM_MIN_VALUE_FOR_EACH_SEGMENT",
    SICK_OP_MODE_MONITOR_TRIGGER_MIN_VALUE_ON_OBJECT                : "MONITOR_TRIGGER_MIN_VALUE_ON_OBJECT",
    SICK_OP_MODE_MONITOR_STREAM_MIN_VERT_DIST_TO_OBJECT             : "MONITOR_STREAM_MIN_VERT_DIST_TO_OBJECT",
    SICK_OP_MODE_MONITOR_TRIGGER_MIN_VERT_DIST_TO_OBJECT            : "MONITOR_TRIGGER_MIN_VERT_DIST_TO_OBJECT",
    SICK_OP_MODE_MONITOR_STREAM_VALUES                              : "MONITOR_STREAM_VALUES",
    SICK_OP_MODE_MONITOR_REQUEST_VALUES                             : "MONITOR_REQUEST_VALUES",
    SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES                         : "MONITOR_STREAM_MEAN_VALUES",
    SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE                     : "MONITOR_STREAM_VALUES_SUBRANGE",
    SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE                : "MONITOR_STREAM_MEAN_VALUES_SUBRANGE",
    SICK_OP_MODE_MONITOR_STREAM_VALUES_WITH_FIELDS                  : "MONITOR_STREAM_VALUES_WITH_FIELDS",
    SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN            : "MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN",
    SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT_FROM_PARTIAL_SCAN : "MONITOR_STREAM_RANGE_AND_REFLECT_FROM_PARTIAL_SCAN",
    SICK_OP_MODE_MONITOR_STREAM_MIN_VALUES_FOR_EACH_SEGMENT_SUBRANGE: "MONITOR_STREAM_MIN_VALUES_FOR_EACH_SEGMENT_SUBRANGE",
    SICK_OP_MODE_MONITOR_NAVIGATION                                 : "MONITOR_NAVIGATION",
    SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT                   : "MONITOR_STREAM_RANGE_AND_REFLECT",
    SICK_OP_MODE_UNKNOWN                                            : "UNKNOWN",
}
make_reversible(sick_lms_measuring_modes)

SICK_BAUD_9600 = 0x42  # 9600 baud
SICK_BAUD_19200 = 0x41  # 19200 baud
SICK_BAUD_38400 = 0x40  # 38400 baud
SICK_BAUD_500K = 0x48  # 500000 baud
SICK_BAUD_UNKNOWN = 0xFF  # Unknown baud rate

sick_lms_baud_codes = {
    9600  : SICK_BAUD_9600,
    19200 : SICK_BAUD_19200,
    38400 : SICK_BAUD_38400,
    500000: SICK_BAUD_500K,
}

sick_lms_bauds = {
    SICK_BAUD_9600 : 9600,
    SICK_BAUD_19200: 19200,
    SICK_BAUD_38400: 38400,
    SICK_BAUD_500K : 500000,
}

SICK_FLAG_AVAILABILITY_DEFAULT = 0x00  # Availability unspecified
SICK_FLAG_AVAILABILITY_HIGH = 0x01  # Highest availability (comparable to LMS types 1 to 5)
SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES = 0x02  # Send real-time indices
SICK_FLAG_AVAILABILITY_DAZZLE_NO_EFFECT = 0x04  # Dazzle evaluation has no effect on switching outputs

sick_lms_flag_availabilities = {
    SICK_FLAG_AVAILABILITY_DEFAULT          : "SICK_FLAG_AVAILABILITY_DEFAULT",
    SICK_FLAG_AVAILABILITY_HIGH             : "SICK_FLAG_AVAILABILITY_HIGH",
    SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES: "SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES",
    SICK_FLAG_AVAILABILITY_DAZZLE_NO_EFFECT : "SICK_FLAG_AVAILABILITY_DAZZLE_NO_EFFECT",
}

restart_codes = [
    "Restart when button actuated",
    "Restart after set time",
    "No restart block",
    "Button switches field set, restart after set time",
    "Button switches field set, no restart block",
    "LMS2xx operates as a slave, restart after set time",
    "LMS2xx operates as a slave, immediate restart",
]


class SickIOException(IOError):
    pass


class SickTimeoutException(TimeoutError):
    pass


class SickConfigException(TimeoutError):
    pass


def get_max_distance(measuring_mode):
    if measuring_mode in (SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE, SICK_MS_MODE_8_OR_80_REFLECTOR, SICK_MS_MODE_8_OR_80_FA_FB_FC):
        return 8.0
    elif measuring_mode in (SICK_MS_MODE_16_REFLECTOR, SICK_MS_MODE_16_FA_FB):
        return 16.0
    elif measuring_mode in (SICK_MS_MODE_32_REFLECTOR, SICK_MS_MODE_32_FA, SICK_MS_MODE_32_IMMEDIATE):
        return 32.0
    else:
        return 0.0

def availability_to_string(availability_code):
    if availability_code == 0:
        return "Default (unspecified)"

    availability_str = ""
    if 0x01 & availability_code:
        availability_str += "Highest"

    if 0x02 & availability_code:
        if len(availability_str) > 0:
            availability_str += ", "
        availability_str += "Real-time indices"

    if 0x04 & availability_code:
        if len(availability_str) > 0:
            availability_str += ", "
        availability_str += "No effect dazzle"

    return availability_str


def restart_to_string(restart_code):
    if restart_code not in restart_codes:
        return "Unknown!"
    else:
        return restart_codes[restart_code]


def temp_field_string(temp_field_code):
    if temp_field_code == 0:
        return "Not used"
    elif temp_field_code == 1:
        return "Belongs to field set no. 1"
    elif temp_field_code == 2:
        return "Belongs to field set no. 2"
    else:
        return "Unknown!"


def subtractive_field_to_string(subt_field_code):
    if subt_field_code == 0:
        return "Not active"
    elif subt_field_code == 1:
        return "Active"
    else:
        return "Unknown!"


def contour_function_to_string(contour_function_code):
    if contour_function_code == 0:
        return "Not active"
    else:
        return "Active, Min object size: %s cm" % contour_function_code


def variant_to_string(sick_variant):
    if sick_variant == SICK_LMS_VARIANT_2XX_TYPE_6:
        return "Standard device (LMS2xx,type 6)"
    elif sick_variant == SICK_LMS_VARIANT_SPECIAL:
        return "Special device (LMS211-/221-S19/-S20)"
    else:
        return "Unknown!"


def status_to_string(sick_status):
    return "Error (possibly fatal)" if sick_status != SICK_STATUS_OK else "OK!"


def is_valid_measuring_units(units):
    return units in sick_lms_measuring_units


def is_valid_scan_angle(scan_angle):
    return scan_angle in sick_lms_scan_angles


def is_valid_scan_resolution(scan_resolution):
    return scan_resolution in sick_lms_scan_resolutions


def is_valid_sensitivity(sensitivity):
    return sensitivity in sick_lms_sensitivities


def is_valid_peak_threshold(peak_threshold):
    return peak_threshold in sick_lms_peak_thresholds


def is_valid_measuring_mode(measuring_mode):
    return measuring_mode in sick_lms_measuring_modes
