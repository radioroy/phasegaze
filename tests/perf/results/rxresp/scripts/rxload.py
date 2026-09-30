import numpy as np
HDR=np.dtype([('lo','<f8'),('prev_lo','<f8'),('gain','<i4'),('mode','<i4'),('hop','<i4'),('nhops','<i4'),
              ('valid','<i4'),('dup','<i4'),('lna','<i4'),('vga','<i4'),('seq','<u4'),('sweep','<u4'),('t','<u8')])
SPAN=131072
REC=np.dtype([('h',HDR),('b','u1',(SPAN,))])
FS=37.3726e6
def load(path):
    r=np.fromfile(path,dtype=REC)
    return r['h'], r['b']
def iq(b):
    """b: (..., SPAN) uint8 -> complex64 (..., 16384, 4)"""
    x=b.view(np.int8).reshape(b.shape[:-1]+(16384,4,2)).astype(np.float32)
    return (x[...,0]+1j*x[...,1]).astype(np.complex64)
