from pupil_apriltags import Detector
import cv2

at_detector = Detector(
   families="tagCustom48h12",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)

at_detector2 = Detector(
   families="tagCircle21h7",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)
img = cv2.imread('./test_data/apriltag_in_apriltag.png',cv2.IMREAD_GRAYSCALE)
# cv2.imshow("Tag",img)


print(at_detector.detect(img))
print(at_detector2.detect(img))
if cv2.waitKey(1) == ord("q"):
    cv2.destroyAllWindows()
    exit(0)




