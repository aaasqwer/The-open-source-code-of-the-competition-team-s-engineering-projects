"""PGD1 lossless RGB565 row codec, independent of lossy JPEG quality."""
import struct
def _signed(x,bits):return x-(1<<bits) if x&(1<<(bits-1)) else x
def encode_row(raw):
    if len(raw)%32:raise ValueError('PGD1 width must be divisible by 16')
    pixels=struct.unpack('<'+'H'*(len(raw)//2),raw);out=bytearray();previous=0
    for start in range(0,len(pixels),16):
        mask=0;codes=[];run=all(p==previous for p in pixels[start:start+16])
        for j,p in enumerate(pixels[start:start+16]):
            dr=_signed(((p>>11)-(previous>>11))&31,5)
            dg=_signed((((p>>5)&63)-((previous>>5)&63))&63,6)
            db=_signed(((p&31)-(previous&31))&31,5)
            if -4<=dr<=3 and -4<=dg<=3 and -2<=db<=1:mask|=1<<j
            codes.append(((dr&7)<<5)|((dg&7)<<2)|(db&3));previous=p
        if run:out.append(2)
        elif mask.bit_count()>=4:
            out.extend(struct.pack('<BH',1,mask))
            for j,p in enumerate(pixels[start:start+16]):out.extend(bytes((codes[j],)) if mask>>j&1 else struct.pack('<H',p))
        else:out.append(0);out.extend(raw[start*2:(start+16)*2])
    return (1,bytes(out)) if len(out)<len(raw) else (0,bytes(raw))
def decode_row_reference(codec,data,width):
    if codec==0:
        if len(data)!=width*2:raise ValueError('Raw row size mismatch')
        return bytes(data)
    if codec!=1 or width%16:raise ValueError('Unsupported PGD1 codec/width')
    out=bytearray(width*2);pos=0;previous=0
    try:
        for start in range(0,width,16):
            kind=data[pos];pos+=1
            if kind==0:
                end=pos+32
                if end>len(data):raise ValueError('Truncated PGD1 raw group')
                out[start*2:(start+16)*2]=data[pos:end];previous=data[end-2]|data[end-1]<<8;pos=end
            elif kind==1:
                mask=data[pos]|data[pos+1]<<8;pos+=2
                for j in range(16):
                    if mask>>j&1:
                        b=data[pos];pos+=1
                        r=((previous>>11)+_signed(b>>5,3))&31
                        g=(((previous>>5)&63)+_signed((b>>2)&7,3))&63
                        blue=((previous&31)+_signed(b&3,2))&31
                        p=(r<<11)|(g<<5)|blue
                    else:p=data[pos]|data[pos+1]<<8;pos+=2
                    offset=(start+j)*2;out[offset]=p&255;out[offset+1]=p>>8;previous=p
            elif kind==2:out[start*2:(start+16)*2]=struct.pack('<H',previous)*16
            else:raise ValueError('Invalid PGD1 group tag')
    except IndexError as exc:raise ValueError('Truncated PGD1 row') from exc
    if pos!=len(data):raise ValueError('Trailing PGD1 bytes')
    return bytes(out)

def decode_row_numpy(codec,data,width):
    if codec==0:return decode_row_reference(codec,data,width)
    if codec!=1 or width%16 or width<=0:raise ValueError('Unsupported PGD1 codec/width')
    try:import numpy as np
    except ImportError:return decode_row_reference(codec,data,width)
    # Parse one descriptor per 16 pixels, then reconstruct all components in
    # vector operations. Literal pixels reset the prefix sum; runs add zero.
    begins=[];masks=[];runs=[];pos=0
    try:
        for _ in range(width//16):
            kind=data[pos];pos+=1
            if kind==0:mask=0;begin=pos;pos+=32
            elif kind==1:
                mask=data[pos]|data[pos+1]<<8;pos+=2;begin=pos;pos+=32-mask.bit_count()
            elif kind==2:mask=65535;begin=0
            else:raise ValueError('Invalid PGD1 group tag')
            if pos>len(data):raise ValueError('Truncated PGD1 row')
            begins.append(begin);masks.append(mask);runs.append(kind==2)
    except IndexError as exc:raise ValueError('Truncated PGD1 row') from exc
    if pos!=len(data):raise ValueError('Trailing PGD1 bytes')
    flags=((np.asarray(masks,dtype=np.uint16)[:,None]>>np.arange(16,dtype=np.uint16))&1).astype(bool)
    lengths=np.where(flags,1,2);offsets=lengths.cumsum(axis=1)-lengths
    positions=np.asarray(begins)[:,None]+offsets
    run=np.repeat(runs,16);positions[np.asarray(runs,dtype=bool)]=0
    flags=flags.ravel();positions=positions.ravel();wire=np.frombuffer(data,dtype=np.uint8)
    first=wire[positions].astype(np.int32);second=wire[np.minimum(positions+1,len(data)-1)].astype(np.int32)
    first[run]=0
    dr=first>>5;dr=np.where(dr>=4,dr-8,dr)
    dg=(first>>2)&7;dg=np.where(dg>=4,dg-8,dg)
    db=first&3;db=np.where(db>=2,db-4,db)
    delta=np.column_stack((dr,dg,db));delta[~flags]=0
    summed=delta.cumsum(axis=0,dtype=np.int32)
    words=first|(second<<8)
    literal=np.column_stack((words>>11,(words>>5)&63,words&31))
    reset=np.zeros((width+1,3),dtype=np.int32);reset[1:]=literal-summed
    last=np.maximum.accumulate(np.where(flags,0,np.arange(1,width+1)))
    value=summed+reset[last]
    decoded=((value[:,0]&31)<<11)|((value[:,1]&63)<<5)|(value[:,2]&31)
    return decoded.astype('<u2').tobytes()

# The packaged DLL is built from pgd1_native.c; Python remains a portable fallback.
def _load_native():
    import ctypes,sys
    from pathlib import Path
    for root in (Path(__file__).resolve().parent,Path(getattr(sys,'_MEIPASS',Path(__file__).resolve().parent))):
        path=root/'pgd1_native.dll'
        if path.is_file():
            try:
                dll=ctypes.CDLL(str(path));fn=dll.pgd1_decode
                fn.argtypes=(ctypes.c_int,ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p,ctypes.c_size_t)
                fn.restype=ctypes.c_int
                return dll,fn
            except OSError:pass
    return None,None

_native_dll,_native_decode=_load_native()
NATIVE_AVAILABLE=_native_decode is not None

def decode_row(codec,data,width):
    if _native_decode is None:return decode_row_numpy(codec,data,width)
    import ctypes
    if width<=0 or width>1920 or width%16:raise ValueError('Unsupported PGD1 width')
    raw=ctypes.create_string_buffer(width*2)
    if _native_decode(codec,bytes(data),len(data),raw,width):raise ValueError('Invalid/truncated PGD1 row')
    return raw.raw

def _frame_function():
    if _native_dll is None:return None
    import ctypes
    try:fn=_native_dll.pgd1_frame
    except AttributeError:return None
    fn.argtypes=(ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p,ctypes.c_size_t,ctypes.c_size_t)
    fn.restype=ctypes.c_int
    return fn
_native_frame=_frame_function()
NATIVE_FRAME_AVAILABLE=_native_frame is not None

def preview_rgb_image(width,height,data):
    if len(data)!=width*height*2:raise ValueError('RGB565 frame length')
    if _native_dll is None or not hasattr(_native_dll,'pgd1_rgb'):
        from protocol import rgb565_to_image
        return rgb565_to_image(width,height,data)
    import ctypes
    from PIL import Image
    fn=_native_dll.pgd1_rgb
    fn.argtypes=(ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p);fn.restype=ctypes.c_int
    out=ctypes.create_string_buffer(width*height*3)
    if fn(data,len(data),out):raise ValueError('RGB565 preview conversion')
    return Image.frombytes('RGB',(width,height),out.raw)

def decode_checked_frame(rows,width,height):
    if not 0<width<=1920 or width%16 or not 0<height<=1080:raise ValueError('Invalid frame geometry')
    if _native_frame is None:
        import zlib
        out=bytearray(width*height*2)
        for row in range(height):
            kind,data,expected=rows[row];raw=decode_row(kind,data,width)
            if zlib.crc32(struct.pack('<H',row)+raw)!=expected:raise ValueError('Preview decoded CRC mismatch')
            out[row*width*2:(row+1)*width*2]=raw
        return bytes(out)
    import ctypes
    chunks=[]
    for row in range(height):
        kind,data,crc=rows[row];chunks.extend((struct.pack('<BHI',kind,len(data),crc),data))
    wire=b''.join(chunks);out=ctypes.create_string_buffer(width*height*2)
    if _native_frame(wire,len(wire),out,width,height):raise ValueError('Preview decompression/CRC failed')
    return out.raw
