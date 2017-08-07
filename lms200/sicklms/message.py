from sicklms.constants import *


class Message:
    """
    A helper class representing important properties of a lms device message
    """
    def __init__(self, payload=b'', checksum=0, buffer=b''):
        self.payload = payload
        self.checksum = checksum
        self.buffer = buffer

        self.full_message = b''

    def append_byte(self, input_byte):
        """Append a byte to the current buffer. Used in _read, _read_8, and _read_16"""
        self.buffer += input_byte

    def make_message(self, length, payload, checksum):
        """From the received bytes, calculate the checksum. Store the input values for later"""
        self.payload = payload

        calc_checksum = Message.crc(self.buffer[:-2])

        if calc_checksum != checksum:
            raise SickIOException("Invalid checksum! found: %s != calculated: %s, %s" % (
                checksum, calc_checksum, repr(self.buffer)))

        self.full_message = DEFAULT_SICK_LMS_MESSAGE_HEADER_BYTE + \
                            DEFAULT_SICK_LMS_HOST_ADDRESS_BYTE + \
                            Message.int_to_byte(length, 2) + \
                            payload + \
                            Message.int_to_byte(checksum, 2)

    def reset(self):
        """Reset the buffer"""
        self.payload = b''
        self.checksum = 0
        self.buffer = b''

    @staticmethod
    def int_to_byte(number, size):
        """Convert an integer to bytes"""
        try:
            return number.to_bytes(size, byteorder='big', signed=False)[::-1]
        except OverflowError as error:
            raise OverflowError("%s is too big to be converted to size %s" % (number, size)) from error

    @property
    def code(self):
        """Get the first byte in the payload if there is one. 0 otherwise"""
        if len(self.payload) > 0:
            return self.payload[0]
        else:
            return 0

    def parse_int(self, index=0, n=1, reversed=True) -> int:
        """
        Parse the payload as an integer
        
        :param index: index to start parsing at
        :param n: number of bytes to parse after index
        :param reversed: flip the order of bytes
        :return: integer representation of the payload 
        """
        if reversed:
            return int.from_bytes(self.payload[index: index + n][::-1], byteorder='big')
        else:
            return int.from_bytes(self.payload[index: index + n], byteorder='big')

    def parse_string(self, index=0, n=None) -> str:
        """
        Parse the payload as a string
        :param index: index to start parsing at
        :param n: number of bytes to parse after index
        :return: string representation of the payload
        """
        if n is None:
            return self.payload[index:].decode()
        else:
            return self.payload[index: index + n].decode()

    @staticmethod
    def crc(data: bytes) -> int:
        """
        Calculate the checksum with the given bytes
        :param data: bytes
        :return: the checksum
        """
        crc16 = 0
        byte = [0, 0]
        for c in data:
            byte[1] = byte[0]
            byte[0] = c

            if crc16 & 0x8000:
                crc16 = (crc16 & 0x7FFF) << 1
                crc16 ^= 0x8005
            else:
                crc16 = crc16 << 1

            crc16 ^= (byte[1] << 8) | byte[0]

        return crc16

    @staticmethod
    def make(payload, address=DEFAULT_SICK_LMS_SICK_ADDRESS) -> bytes:
        """
        Create a bytes message from the given payload and address
        
        :param payload: data to send 
        :param address: address to send data to
        :return: the full message to send
        """

        # message header
        data = DEFAULT_SICK_LMS_MESSAGE_HEADER_BYTE + Message.int_to_byte(address, 1)

        # length and payload
        data += Message.int_to_byte(len(payload), 2)
        data += payload

        # checksum
        checksum = Message.crc(data)
        data += Message.int_to_byte(checksum, 2)

        return data

    def __str__(self):
        return "%s(buffer=%s)" % (
            self.__class__.__name__, self.buffer
        )

