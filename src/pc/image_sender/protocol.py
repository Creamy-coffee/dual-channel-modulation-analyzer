import struct,zlib
def make_packets(pixels,w,h,image_id):
    crc=zlib.crc32(pixels);out=[]
    for i,at in enumerate(range(0,len(pixels),32)):
        payload=pixels[at:at+32]
        header=struct.pack('<8sBBHHHIIIHH',b'V32IMG1!',1,1,w,h,32,image_id,crc,len(pixels),i,len(payload))
        assert len(header)==32
        packet=header+payload.ljust(32,b'\0')
        out.append(packet+struct.pack('<I',zlib.crc32(packet)))
    return out

def symbols_for(packets):
    symbols=bytearray()
    for packet in packets:
        for byte in b'\x55'*32+packet:
            for i in range(7,-1,-1):
                bit=(byte>>i)&1;symbols.extend((bit,1-bit))
    return bytes(symbols)
