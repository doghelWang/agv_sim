import ctypes
import os

path = '/data/data/com.termux/files/home/libonnxruntime.so'
print('Loading:', path)
try:
    c = ctypes.CDLL(path)
    print('SUCCESS! Loaded ORT:', c)
except Exception as e:
    print('ERROR:', e)
