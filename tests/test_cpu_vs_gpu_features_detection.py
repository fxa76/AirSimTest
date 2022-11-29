import cv2
import time

# test feature detection with cpu vs gpu

cv2.cuda.setDevice(0)
cuda_orb = cv2.cuda.ORB_create(nlevels=7)

#read image from disk
img1 = cv2.imread("./test_data/target2.png")

start = time.time()

src = cv2.cuda_GpuMat()
src.upload(img1)
gray_frame = cv2.cuda.cvtColor(src, cv2.COLOR_BGR2GRAY)
# img = gray_frame.download()
# cv2.imshow("baseball", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

kp1, descs1 = cuda_orb.detectAndComputeAsync(gray_frame, None)

end = time.time()

print(end-start)

k= cuda_orb.convert(kp1)
# img = cv2.drawKeypoints(img, k, None)
# cv2.imshow("Image", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# init
orb = cv2.ORB_create()
start = time.time()
gray_frame = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
kp1, descs1 =orb.detectAndCompute(gray_frame, None)

end = time.time()

print(end-start)