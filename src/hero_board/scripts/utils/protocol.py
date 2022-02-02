import socket
import typing


def var_len_proto_send(data: list) -> bytes:
    buffer = bytearray()
    buffer.append(0xff)
    buffer.append(len(data) | 0b11000000)
    buffer.extend(data)
    buffer.append(sum(buffer) % 256)
    return buffer


temp = bytearray()


def var_len_proto_recv(data: bytes) -> typing.List[bytearray]:
    global temp
    temp.extend(data)
    output = []
    while len(temp) > 2:  # read at least 3 bytes
        for i in range(len(temp) - 2):
            if temp[i] == 0xFF and temp[i + 1] & 0b11000000 == 0b11000000:  # check the header
                count = temp[i + 1] & 0b111111
                expected_len = 2 + count + 1  # 1 header + 1 count byte + ... + 1 checksum
                # if temp's length is greater than the expected packet length
                if len(temp) >= i + expected_len:
                    slc = temp[i:i + expected_len]

                    # remove bytes already read
                    del temp[:i + expected_len]

                    if sum(slc[:-1]) % 256 == slc[-1]:
                        output.append(slc[2:-1])
                    break
                else:
                    # not enough bytes
                    del temp[:i]  # remove bytes already read
                    return output
        else:  # no single matching byte, remove all
            temp.clear()
    return output