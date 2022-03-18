from protocol import var_len_proto_recv, var_len_proto_send, Opcode
from random import randint


def gen_var_send_test(opcode: Opcode, data: list):
    count = len(data)
    count = count | opcode.value

    buffer = [0xff, count]
    buffer.extend(data)
    buffer.append(sum(buffer) % 256)
    return bytes(buffer)


# ---- client test -----------------
assert var_len_proto_send(Opcode.DIRECT_DRIVE, [1, 2, 3]) == bytes([255, 67, 1, 2, 3, 72])
assert var_len_proto_send(Opcode.DIRECT_DRIVE, [0, 0, 0]) == bytes([255, 67, 0, 0, 0, 66])
assert var_len_proto_send(Opcode.PID, [1, 2, 3]) == bytes([255, 67+64, 1, 2, 3, 72+64])
for i in range(50):
    count = randint(1, 16)
    data = [randint(0, 255) for i in range(count)]
    assert var_len_proto_send(Opcode.DIRECT_DRIVE, data) == gen_var_send_test(Opcode.DIRECT_DRIVE, data)


# ---- server test
assert var_len_proto_recv(bytes([255, 195, 1, 2, 3, 200])) == [(Opcode.FEEDBACK, bytes([1, 2, 3]))] # Opcode.FEEDBACK = 0b11000000 = 192
assert var_len_proto_recv(bytes([255, 67, 1, 2, 3, 72])) == [(Opcode.DIRECT_DRIVE, bytes([1, 2, 3]))] # Opcode.DIRECT_DRIVE = 0b01000000 = 64
for i in range(50):
    count = randint(1, 16)
    data = [randint(0, 255) for i in range(count)]
    assert var_len_proto_recv(gen_var_send_test(Opcode.FEEDBACK, data)) == [(Opcode.FEEDBACK, bytes(data))]


# ---- incomplete data test
assert var_len_proto_recv(bytes([254])) == []
assert var_len_proto_recv(bytes([])) == []
assert var_len_proto_recv(bytes([255, 195, 1, 2, 3, 199])) == []  # checksum mismatch
assert var_len_proto_recv(bytes([255, 195])) == []  # segmented data
assert var_len_proto_recv(bytes([1, 2, 3, 200])) == [(Opcode.FEEDBACK, bytes([1, 2, 3]))]
assert var_len_proto_recv(bytes([255, 195, 2, 2, 3])) == []
assert var_len_proto_recv(bytes([201])) == [(Opcode.FEEDBACK, bytes([2, 2, 3]))]
assert var_len_proto_recv(bytes([255])) == []
assert var_len_proto_recv(bytes([195])) == []
assert var_len_proto_recv(bytes([2])) == []
assert var_len_proto_recv(bytes([2, 3])) == []
assert var_len_proto_recv(bytes([201])) == [(Opcode.FEEDBACK, bytes([2, 2, 3]))]
assert var_len_proto_recv(bytes([255, 64, 1, 2, 3, 200])) == []  # count byte mismatch

assert var_len_proto_recv(bytes([255, 195, 1, 2, 3, 200] * 2)) == [(Opcode.FEEDBACK, bytes([1, 2, 3]))] * 2  # two packages all at once


# ---- back and forth test
assert var_len_proto_recv(var_len_proto_send(Opcode.DIRECT_DRIVE, [1, 2, 3])) == [(Opcode.DIRECT_DRIVE, bytes([1, 2, 3]))]
for i in range(50):
    count = randint(1, 16)
    data = [randint(0, 255) for i in range(count)]
    assert var_len_proto_recv(var_len_proto_send(Opcode.DIRECT_DRIVE, data)) == [(Opcode.DIRECT_DRIVE, bytes(data))]

