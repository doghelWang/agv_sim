import ctypes

print("=== Testing Native Bionic GPU Access in Termux ===")
try:
    cl = ctypes.CDLL("/vendor/lib64/libOpenCL.so")
    print("SUCCESS: Loaded /vendor/lib64/libOpenCL.so in Termux!")
    clGetPlatformIDs = getattr(cl, clGetPlatformIDs, None)
    if clGetPlatformIDs:
        num = ctypes.c_uint32()
        res = clGetPlatformIDs(0, None, ctypes.byref(num))
        print(f"clGetPlatformIDs returned {res}, platforms count: {num.value}")
except Exception as e:
    print("OpenCL error:", e)

try:
    vk = ctypes.CDLL("/vendor/lib64/hw/vulkan.adreno.so")
    print("SUCCESS: Loaded /vendor/lib64/hw/vulkan.adreno.so in Termux!")
except Exception as e:
    print("Vulkan error:", e)
