from sicklms.constants import *


class Message:
    def __init__(self, payload=b'', checksum=0, buffer=b''):
        self.payload = payload
        self.checksum = checksum
        self.buffer = buffer

        self.full_message = b''

    def append_byte(self, input_byte):
        self.buffer += input_byte

    def make_message(self, length, payload, checksum):
        self.payload = payload

        calc_checksum = Message.crc(self.buffer[:-2])

        if calc_checksum != checksum:
            raise SickIOException("Invalid checksum! found: %s != calculated: %s, %s" % (
                checksum, calc_checksum, repr(self.buffer)))

        self.full_message = b'\x20\x80' + Message.int_to_byte(length, 2) + payload + Message.int_to_byte(checksum, 2)

    def reset(self):
        self.payload = b''
        self.checksum = 0
        self.buffer = b''

    def make_buffer(self, address=DEFAULT_SICK_LMS_SICK_ADDRESS) -> bytes:
        data = b'\x02' + Message.int_to_byte(address, 1)
        data += Message.int_to_byte(len(self.payload), 2)
        data += self.payload
        self.checksum = Message.crc(data)
        data += Message.int_to_byte(self.checksum, 2)

        self.buffer = data
        return data

    @staticmethod
    def int_to_byte(number, size):
        try:
            return number.to_bytes(size, byteorder='big', signed=False)[::-1]
        except OverflowError as error:
            raise OverflowError("%s is too big to be converted to size %s" % (number, size)) from error

    @property
    def code(self):
        if len(self.payload) > 0:
            return self.payload[0]
        else:
            return 0

    def parse_int(self, index=0, n=1, reversed=True) -> int:
        if reversed:
            return int.from_bytes(self.payload[index: index + n][::-1], byteorder='big')
        else:
            return int.from_bytes(self.payload[index: index + n], byteorder='big')

    def parse_string(self, index=0, n=None) -> str:
        if n is None:
            return self.payload[index:].decode()
        else:
            return self.payload[index: index + n].decode()

    @staticmethod
    def crc(data):
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

    def __str__(self):
        return "%s(buffer=%s)" % (
            self.__class__.__name__, self.buffer
        )
