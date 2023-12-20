# coding=utf-8
import sys
import uuid

if __name__ == "__main__" :
    if len(sys.argv) < 2 :
        print("Usage: python erofs_uuid.py image offset")
        sys.exit()

    image_path = sys.argv[1]
    if sys.argv == 3 :
        offset = int(sys.argv[2])
    else :
        offset = 1024

    image_fo = open(image_path, "rb")
    image_fo.seek(offset)

    out_info = image_fo.read(4)
    magic = int("0x%s" % (''.join(['%02X' % b for b in reversed(out_info)])), 16)

    if magic != 0xE0F5E1E2 :
        print("magic number is wrong")
        sys.exit()

    image_fo.seek(offset + 48)
    out_info = image_fo.read(16)
    uuid_hex = "0x%s" % (''.join(['%02X' % b for b in out_info]))
    print(uuid.UUID(int=int(uuid_hex, 16)))

    image_fo.close()
