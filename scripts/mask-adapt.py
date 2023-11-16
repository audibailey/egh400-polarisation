import cv2
import numpy as np
import polanalyser

# workaround because no polarisation camera
img_blended = cv2.imread('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/inperson-test/test.png')

# convert to grey
img_grey = cv2.cvtColor(img_blended, cv2.COLOR_BGR2GRAY)
cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/results/inperson/grey.png', img_grey)

# create threshold
adaptive = cv2.adaptiveThreshold(img_grey, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 51, 9)
cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/results/inperson/thres.png', adaptive)

# modify threshold to be dilated and eroded - this helps connect the rectangle perimeter
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
opening = cv2.morphologyEx(adaptive, cv2.MORPH_CLOSE, kernel, iterations=5)
cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/results/inperson/smooth.png', opening)

# find contours
contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
image_contour = np.zeros_like(img_blended)
contour_details = (0, 0, 0, 0)
for contour in contours:
    area = cv2.contourArea(contour)
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.015 * peri, True)
    # looking for a single large rectangle
    if len(approx) == 13 and area >= 5000 and contour_details == (0, 0, 0, 0):
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image_contour, (x, y), (x + w, y + h), (255, 255, 255), -1)
        contour_details = (x, y, x + w, y + h)
cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/results/inperson/image-contour.png', image_contour)
print(contour_details)

# make mask
lower_white = np.array([0, 0, 255])
upper_white = np.array([255, 255, 255])
mask = cv2.inRange(image_contour, lower_white, upper_white)
cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/results/inperson/mask.png', mask)

# apply mask
result = cv2.bitwise_and(img_blended, img_blended, mask=mask)
cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/results/inperson/result.png', result)

resize = result[contour_details[1]:contour_details[3], contour_details[0]:contour_details[2]]
cv2.imwrite('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/results/inperson/resize_test.png', resize)

img_blended = cv2.imread('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/inperson-test/test.png', 0)
# calculate polarisation after resizing images to mask
# requires reimporting because flags - with a normal camera this wont be needed
imgs = polanalyser.demosaicing(img_blended, polanalyser.COLOR_PolarMono)
angles = np.deg2rad([0, 45, 90, 135])
image_list = [
    imgs[0][contour_details[1]:contour_details[3], contour_details[0]:contour_details[2]],
    imgs[1][contour_details[1]:contour_details[3], contour_details[0]:contour_details[2]],
    imgs[2][contour_details[1]:contour_details[3], contour_details[0]:contour_details[2]],
    imgs[3][contour_details[1]:contour_details[3], contour_details[0]:contour_details[2]]
]

img_stokes = polanalyser.calcStokes(image_list, angles)

img_intensity = polanalyser.cvtStokesToIntensity(img_stokes)
img_dolp = polanalyser.cvtStokesToDoLP(img_stokes)
img_aolp = polanalyser.cvtStokesToAoLP(img_stokes)

img_dolp_u8 = np.clip(img_dolp * 255, 0, 255).astype(np.uint8)
img_aolp_u8 = polanalyser.applyColorToAoLP(img_aolp)

# export images
name = '/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/results/inperson/export'
cv2.imwrite(f"{name}_intensity.png", img_intensity)
cv2.imwrite(f"{name}_DoLP.png", img_dolp_u8)
cv2.imwrite(f"{name}_AoLP.png", img_aolp_u8)

# calculate polarisation after averaging images
# DoLP = Mean single number out
# AoLP = Median single number out
print(np.average(img_dolp_u8))
print(np.median(img_aolp_u8))
# img_0_mean = np.full(image_list[0].shape, np.average(image_list[0]))
# img_45_mean = np.full(image_list[1].shape, np.average(image_list[1]))
# img_90_mean = np.full(image_list[2].shape, np.average(image_list[2]))
# img_135_mean = np.full(image_list[3].shape, np.average(image_list[3]))
# image_mean_list = [img_0_mean, img_45_mean, img_90_mean, img_135_mean]
#
# img_mean_stokes = polanalyser.calcStokes(image_mean_list, angles)
#
# img_mean_intensity = polanalyser.cvtStokesToIntensity(img_mean_stokes)
# img_mean_dolp = polanalyser.cvtStokesToDoLP(img_mean_stokes)
# img_mean_aolp = polanalyser.cvtStokesToAoLP(img_mean_stokes)
#
# img_mean_dolp_u8 = np.clip(img_mean_dolp * 255, 0, 255).astype(np.uint8)
# img_mean_aolp_u8 = polanalyser.applyColorToAoLP(img_mean_aolp)
#
# # export
# name = '/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/results/inperson/export_mean'
# cv2.imwrite(f"{name}_intensity.png", img_mean_intensity)
# cv2.imwrite(f"{name}_DoLP.png", img_mean_dolp_u8)
# cv2.imwrite(f"{name}_AoLP.png", img_mean_aolp_u8)
