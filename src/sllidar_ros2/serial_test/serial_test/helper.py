import numpy as np
import struct

class Helper:
    def uint8arr_to_int16(byte1, byte2, little_endian=True):
        if little_endian:
            packed = struct.pack('BB', byte1, byte2)
        else:
            packed = struct.pack('BB', byte2, byte1)
        return struct.unpack('<h' if little_endian else '>h', packed)[0]
    
    def int16_to_uint8arr(value, little_endian=True):
        packed = struct.pack('<h' if little_endian else '>h', value)
        return struct.unpack('BB', packed)
    
    def uint8arr_to_int32(byte1, byte2, byte3, byte4, little_endian=True):
        if little_endian:
            packed = struct.pack('BBBB', byte1, byte2, byte3, byte4)
        else:
            packed = struct.pack('BBBB', byte4, byte3, byte2, byte1)
        return struct.unpack('<i' if little_endian else '>i', packed)[0]
    
    def int32_to_uint8arr(value, little_endian=True):
        packed = struct.pack('<i' if little_endian else '>i', value)
        return struct.unpack('BBBB', packed)