import socket
import typing


def var_len_proto_send(data: list) -> bytes:
    count = len(data)
    count = count | 0b11000000

    buffer = [0xff, count]
    buffer.extend(data)
    buffer.append(sum(buffer) % 256)
    return bytes(buffer)


temp = bytes()


def var_len_proto_recv(data: bytes) -> typing.List[bytes]:
    global temp
    temp += data
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
                    temp = temp[i + expected_len:]

                    if sum(slc[:-1]) % 256 == slc[-1]:
                        output.append(slc[2:-1])
                    break
                else:
                    # not enough bytes
                    temp = temp[i:]  # remove bytes already read
                    return output
        else:  # no single matching byte, remove all
            temp = bytes()
    return output