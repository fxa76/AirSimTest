#!/usr/bin/python3
# 2017.11.26 23:27:12 CST

## Find object by orb features matching

import numpy as np
import cv2


def find_homography(img1, img2):
    MIN_MATCH_COUNT = 4

    ## Create ORB object and BF object(using HAMMING)
    orb = cv2.cuda.ORB_create(nlevels=7)


    src1 = cv2.cuda_GpuMat()
    src1.upload(img1)
    matSrc1= cv2.cuda.cvtColor(src1, cv2.COLOR_BGR2GRAY)

    src2 = cv2.cuda_GpuMat()
    src2.upload(img2)
    matSrc2= cv2.cuda.cvtColor(src2, cv2.COLOR_BGR2GRAY)


    ## Find the keypoints and descriptors with ORB
    kpts1, descs1 = orb.detectAndComputeAsync(matSrc1,None)
    kpts2, descs2 = orb.detectAndComputeAsync(matSrc2,None)

    ## match descriptors and sort them in the order of their distance
    bf = cv2.cuda.DescriptorMatcher_createBFMatcher(cv2.NORM_HAMMING)
    matches = bf.match(descs1, descs2)
    # dmatches = sorted(matches, key = lambda x:x.distance)

    # print(min(m.distance for m in dmatches))
    # store all the good matches as per Lowe's ratio test
    # MIN_DIST_THRESHOLD = 1000
    # good = []
    # for m in dmatches:
    #     if m.distance < MIN_DIST_THRESHOLD :
    #         good.append(m)

    kpts1 = orb.convert(kpts1)
    kpts2 = orb.convert(kpts2)

    ## extract the matched keypoints
    src_pts  = np.float32([kpts1[m.queryIdx].pt for m in matches]).reshape(-1,1,2)
    dst_pts  = np.float32([kpts2[m.trainIdx].pt for m in matches]).reshape(-1,1,2)

    ## find homography matrix and do perspective transform
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    h,w = img1.shape[:2]
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)

    ## draw found regions
    img2 = cv2.polylines(img2, [np.int32(dst)], True, (0,0,255), 1, cv2.LINE_AA)
    #cv2.imshow("found", img2)

    ## draw match lines
    res = cv2.drawMatches(img1, kpts1, img2, kpts2, matches,None,flags=2)

    return res

if __name__ == '__main__':
    imgPath1 = "./test_data/target2.png"# query image (small object)
    imgPath2 = "./test_data/view2.png" # train image (large scene)
    img1 = cv2.imread(imgPath1)
    img2 = cv2.imread(imgPath2)
    res = find_homography(img1,img2)
    cv2.imshow("orb_match", res);
    cv2.waitKey();cv2.destroyAllWindows()