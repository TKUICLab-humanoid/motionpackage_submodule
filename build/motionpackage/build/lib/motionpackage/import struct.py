import struct

data = b'*\x00\x00\x00\xc3\xf5H@\x61'
unpacked = struct.unpack('i f c', data)
print(unpacked)  # 输出：(42, 3.14, b'a')
