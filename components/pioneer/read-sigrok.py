import sys
import zipfile

data = []
with zipfile.ZipFile(sys.argv[1], "r") as archive:
    # Sort/merge the logic analyzer files accounting for no leading zeroes, e.g. logic-1-1, logic-1-2, ..., logic-1-10, logic-1-11, ...
    file_names = [x for x in archive.namelist() if x.startswith("logic-1-")]
    for file_name in file_names:
        data += archive.read(file_name)

HIGH = 0xFF
LOW = 0xFD

last = LOW
count = 0
i = 0
j = 0
msg = 0
msg_bytes = []
for b in data:
    if b == last:
        count += 1
    elif count > 5000:
        if count > 30_000:
            # print(f"discarding leader byte: {count}, {count/24:0.1f}us @ {i/24:0.1f}us")
            pass
        elif 10_000 < count < 20_000:
            # print(f"discarding delimiter: {count}, {count/24:0.1f}us @ {i/24:0.1f}us")
            pass
        else:
            # print(f"count: {count}, {count/24:0.1f}us @ {i/24:0.1f}us")
            bit = 0
            if count > 20_000:
                bit = 1
            msg |= bit << (i % 8)
            if i % 8 == 7:
                if j == 14:
                    j = 0
                print(f"{j:02d}: 0x{msg:02x}")
                msg_bytes.append(msg)
                msg = 0
                j += 1
            i += 1
        count = 0
        last = b
    else:
        count = 1
        last = b

sum = 0x0F
for i in range(0, len(msg_bytes)):
    if i == 13 or i == 27:
        if sum % 256 != msg_bytes[i]:
            print(f"checksum mismatch: 0x{sum%256:02x} rcvd: 0x{msg_bytes[i]:02x}")
        else:
            print(f"checksums match")
        sum = 0
        continue
    sum += msg_bytes[i]
