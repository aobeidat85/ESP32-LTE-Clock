"""Independent NumPy LTE synchronization vectors, TS 36.211 section 6.11."""
import numpy as np
NFFT=128
BINS=np.r_[np.arange(-31,0),np.arange(1,32)]%NFFT

def pss(nid2):
    u=(25,29,34)[nid2]
    n=np.arange(63)
    return np.delete(np.exp(-1j*np.pi*u*n*(n+1)/63),31)

def symbol(sequence):
    bins=np.zeros(NFFT,complex);bins[BINS]=sequence
    return np.fft.ifft(bins)

def sss(nid1,nid2,half):
    def base(taps):
        x=np.zeros(31,int);x[4]=1
        for n in range(26): x[n+5]=sum(x[n+t] for t in taps)%2
        return 1-2*x
    s,c,z=base([0,2]),base([0,3]),base([0,1,2,4])
    qprime=nid1//30;q=(nid1+qprime*(qprime+1)//2)//30
    mprime=nid1+q*(q+1)//2;m0=mprime%31;m1=(m0+mprime//31+1)%31
    n=np.arange(31);a,b=(m0,m1) if half==0 else (m1,m0)
    result=np.empty(62)
    result[0::2]=s[(n+a)%31]*c[(n+nid2)%31]
    result[1::2]=s[(n+b)%31]*c[(n+nid2+3)%31]*z[(n+(a%8))%31]
    return result
