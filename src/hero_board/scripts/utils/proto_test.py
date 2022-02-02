from protocol import var_len_proto_recv, var_len_proto_send
from random import randint


def gen_var_send_test(data: list):
    count = len(data)
    count = count | 0b11000000

    buffer = [0xff, count]
    buffer.extend(data)
    buffer.append(sum(buffer) % 256)
    return bytes(buffer)


# ---- client test -----------------
assert var_len_proto_send([1, 2, 3]) == bytes([255, 195, 1, 2, 3, 200])
for i in range(50):
    count = randint(1, 16)
    data = [randint(0, 255) for i in range(count)]
    assert var_len_proto_send(data) == gen_var_send_test(data)


# ----- server test
assert var_len_proto_recv(bytes([255, 195, 1, 2, 3, 200])) == [bytes([1, 2, 3])]
for i in range(50):
    count = randint(1, 16)
    data = [randint(0, 255) for i in range(count)]
    assert var_len_proto_recv(gen_var_send_test(data)) == [bytes(data)]

# --- incomplete data test
assert var_len_proto_recv(bytes([254])) == []
assert var_len_proto_recv(bytes([])) == []
assert var_len_proto_recv(bytes([255, 195, 1, 2, 3, 199])) == []  # checksum mismatch
assert var_len_proto_recv(bytes([255, 195])) == []  # segmented data
assert var_len_proto_recv(bytes([1, 2, 3, 200])) == [bytes([1, 2, 3])]
assert var_len_proto_recv(bytes([255, 195, 2, 2, 3])) == []
assert var_len_proto_recv(bytes([201])) == [bytes([2, 2, 3])]
assert var_len_proto_recv(bytes([255])) == []
assert var_len_proto_recv(bytes([195])) == []
assert var_len_proto_recv(bytes([2])) == []
assert var_len_proto_recv(bytes([2, 3])) == []
assert var_len_proto_recv(bytes([201])) == [bytes([2, 2, 3])]
assert var_len_proto_recv(bytes([255, 64, 1, 2, 3, 200])) == []  # count byte mismatch

assert var_len_proto_recv(bytes([255, 195, 1, 2, 3, 200] * 2)) == [bytes([1, 2, 3])] * 2  # two packages all at once
