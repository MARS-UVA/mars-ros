import socket
import typing
from enum import Enum


# All the possible opcodes stored as bytes
class Opcode(Enum):
    STOP = 0b00000000
    DIRECT_DRIVE = 0b01000000
    PID = 0b10000000
    FEEDBACK = 0b11000000

opcode_options = [e.value for e in Opcode]


def var_len_proto_send(opcode: Opcode, data: list) -> bytes:
    buffer = bytearray()
    buffer.append(0xff)
    buffer.append(len(data) | opcode.value)
    buffer.extend(data)
    buffer.append(sum(buffer) % 256)
    return buffer


temp = bytearray()


def var_len_proto_recv(data: bytes) -> typing.List[typing.Tuple[int, bytearray]]: # returns (Opcode, Data)
    global temp
    temp.extend(data)
    output = []
    while len(temp) > 2:  # read at least 3 bytes
        for i in range(len(temp) - 2):
            opcode_value = temp[i + 1] & 0b11000000
            if temp[i] == 0xFF and opcode_value in opcode_options:  # check the header
                count = temp[i + 1] & 0b111111
                expected_len = 2 + count + 1  # 1 header + 1 count byte + ... + 1 checksum
                # if temp's length is greater than the expected packet length
                if len(temp) >= i + expected_len:
                    slc = temp[i:i + expected_len]

                    # remove bytes already read
                    del temp[:i + expected_len]

                    if sum(slc[:-1]) % 256 == slc[-1]:
                        output.append( (Opcode(opcode_value), slc[2:-1]) )
                    break
                else:
                    # not enough bytes
                    del temp[:i]  # remove bytes already read
                    return output
        else:  # no single matching byte, remove all
            temp.clear()
    return output
