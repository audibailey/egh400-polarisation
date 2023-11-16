import cv2
import numpy as np
import polanalyser

# img_0 = cv2.imread('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/test-images/DSC02035.JPG', 0)
# img_45 = cv2.imread('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/test-images/DSC02036.JPG', 0)
# img_90 = cv2.imread('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/test-images/DSC02037.JPG', 0)
# img_135 = cv2.imread('/Users/audibailey/Documents/Education/QUT/Semester 12/Theeesis/test-images/DSC02038.JPG', 0)

img = cv2.imread('/Users/audibailey/repos/thesis-ws/inperson/az:-1_al:-1.png', 0)

# print(img.shape)

name = '/Users/audibailey/repos/thesis-ws/scripts/tmp/export'
img_list = polanalyser.demosaicing(img, polanalyser.COLOR_PolarMono)
cv2.imwrite(f"{name}_1.png", img_list[0])
cv2.imwrite(f"{name}_2.png", img_list[1])
cv2.imwrite(f"{name}_3.png", img_list[2])
cv2.imwrite(f"{name}_4.png", img_list[3])


# cv2.imwrite('/Users/audibailey/repos/thesis-ws/tmp/2023-10-23T18:38:21.390783/yee.png', img_list[0])
angles = np.deg2rad([0, 45, 90, 135])

img_stokes = polanalyser.calcStokes(img_list, angles)
img_intensity = polanalyser.cvtStokesToIntensity(img_stokes)
img_dolp = polanalyser.cvtStokesToDoLP(img_stokes)
img_aolp = polanalyser.cvtStokesToAoLP(img_stokes)

img_dolp_u8 = np.clip(img_dolp * 255, 0, 255).astype(np.uint8)
img_aolp_u8 = polanalyser.applyColorToAoLP(img_aolp)

# Export images
cv2.imwrite(f"{name}_intensity.png", img_intensity)
cv2.imwrite(f"{name}_DoLP.png", img_dolp_u8)
cv2.imwrite(f"{name}_AoLP.png", img_aolp_u8)
