import cv2
import numpy as np

## workaround because no polarisation camera
img_0 = cv2.imread('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/img-2/000.JPG')
img_45 = cv2.imread('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/img-2/045.JPG')
img_90 = cv2.imread('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/img-2/090.JPG')
img_135 = cv2.imread('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/img-2/135.JPG')

imgs = [img_45, img_90, img_135]
img_blended = img_0

for img in imgs:
	img_blended = cv2.add(img_blended, img)

# cv2.medianBlur to reduce noise if needed

# convert to hsv
img_hsv = cv2.cvtColor(img_blended, cv2.COLOR_BGR2HSV)

# red mask
# TODO: dynamic red threshold based on image pixel red density
# adaptiveThreshold
lower_red_thres = np.array([0, 100, 100]) #np.array([170, 128, 128])
upper_red_thres = np.array([10, 255, 255]) #np.array([180, 255, 255])
mask_0 = cv2.inRange(img_hsv, lower_red_thres, upper_red_thres)
lower_red_thres = np.array([140, 100, 100]) #np.array([0, 128, 128])
upper_red_thres = np.array([180, 255, 255]) #np.array([20, 255, 255])
mask_1 = cv2.inRange(img_hsv, lower_red_thres, upper_red_thres)

red_mask = mask_0 + mask_1
img_mask = cv2.bitwise_and(img_hsv, img_hsv, mask=red_mask)
cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/img-2/mask-non-kern.png', img_mask)


# smooth out the image
kernel = np.ones((10, 10), np.uint8)
red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

# red_count = cv2.countNonZero(red_mask)
# fraction = red_count / (width * height)
# print(f"{fraction:.2%}")

img_mask = cv2.bitwise_and(img_hsv, img_hsv, mask=red_mask)
cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/img-2/mask.png', img_mask)

# # convert images
# img_bgr = cv2.cvtColor(img_mask, cv2.COLOR_HSV2BGR)
# img_grey = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

# # remove all non-red
# img_bgr[:, :, 0] = np.zeros([img_bgr.shape[0], img_bgr.shape[1]])
# img_bgr[:, :, 1] = np.zeros([img_bgr.shape[0], img_bgr.shape[1]])

# # extract blob
# params = cv2.SimpleBlobDetector_Params()
# params.minThreshold = 10
# params.maxThreshold = 100
# params.filterByColor = True
# params.blobColor = 255
# params.filterByArea = True
# params.minArea = 10
# params.filterByCircularity = True
# params.minCircularity = 0.1
# detector = cv2.SimpleBlobDetector_create(params)

# # # Detect blobs in the mask
# keypoints = detector.detect(img_bgr)
# print(keypoints)

# # # Draw detected blobs on the original image
# result_image = cv2.drawKeypoints(img_bgr, keypoints, np.array([]), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/img-2/yote.png', result_image)

# draw contour on input
# result = img_blended.copy()
# contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
# for contour in contours:
#     # calculate area
#     area = cv2.contourArea(contour)
#     if area > 1000:
#         # try determine circle
#         epsilon = 0.02 * cv2.arcLength(contour, True)
#         approx = cv2.approxPolyDP(contour, epsilon, True)

#         if len(approx) >= 5: 
#             # calculate new circle
#             (x, y), radius = cv2.minEnclosingCircle(contour)
#             center = (int(x), int(y))

#             # create circle mask
#             mask = cv2.circle(np.zeros((result.shape[0], result.shape[1]), dtype=np.uint8), center, int(radius), (255, 255 , 255), -1) 
#             yeet = cv2.circle(result, center, int(radius), (255, 255 , 255), 1) 
#             cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/img-2/final.png', yeet)
#             color = np.full_like(result, (255,255,255)) 
#             masked_img = cv2.bitwise_and(result, result, mask=mask) 
#             masked_color = cv2.bitwise_and(color, color, mask=255-mask)
            
#             # apply mask and save
#             result = cv2.add(masked_img, masked_color)
#             cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/img-2/test.png', result)
#             break