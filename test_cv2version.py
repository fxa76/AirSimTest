import cv2

print("cv2 version : {}".format(cv2.__version__))
print("Number of cuda devices: {}".format(cv2.cuda.getCudaEnabledDeviceCount()))
