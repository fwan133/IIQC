import cv2
import numpy as np
import matplotlib.pyplot as plt


def estimateBlurMap(image, patch_size):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Calculate the laplacian
    laplacian = cv2.Laplacian(gray_image, cv2.CV_64F)
    global_avg = np.log10(np.std(laplacian))

    # Calculate the local std deviation of laplacian
    nx = image.shape[1] // patch_size
    ny = image.shape[0] // patch_size
    # Initialise blur map
    log_blur_map = np.zeros((ny, nx))
    for i in range(ny):
        for j in range(nx):
            # Extract the patch
            patch = laplacian[i * patch_size:(i + 1) * patch_size, j * patch_size:(j + 1) * patch_size]
            log_blur_map[i, j] = np.log10(np.std(patch))
    # Normalise the blurMap
    local_max = np.max(np.ma.masked_invalid(log_blur_map))
    local_min = np.min(np.ma.masked_invalid(log_blur_map))
    if (local_max - global_avg) > (global_avg - local_min):
        local_min = global_avg * 2 - local_max
    else:
        local_max = global_avg * 2 - local_min
    normalised_blur_map = 1 - (log_blur_map - local_min) / (local_max - local_min)
    return normalised_blur_map


# Import image
# image = cv2.imread('../data/test/exposure/DJI_0014.JPG')
image1 = cv2.imread('../../data/images/train_001.JPG')
blurmap1 = estimateBlurMap(image1, 17)
image2 = cv2.imread('../../data/test/exposure/DJI_0014.JPG')
blurmap2 = estimateBlurMap(image2, 17)
image3 = cv2.imread('../../data/test/exposure/DJI_0016.JPG')
blurmap3 = estimateBlurMap(image3, 17)
image4 = cv2.imread('../../data/test/exposure/DJI_0026.JPG')
blurmap4 = estimateBlurMap(image4, 17)


# Plot the figure
plt.figure(figsize=(16, 6))
plt.subplot(2, 4, 1)
# plt.title("Original Image")
plt.imshow(cv2.cvtColor(image1, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.subplot(2, 4, 2)
# plt.title("Original Image")
plt.imshow(cv2.cvtColor(image2, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.subplot(2, 4, 3)
# plt.title("Original Image")
plt.imshow(cv2.cvtColor(image3, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.subplot(2, 4, 4)
# plt.title("Original Image")
plt.imshow(cv2.cvtColor(image4, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.subplot(2, 4, 5)
# plt.title("Normalised Blur Map")
plt.imshow(blurmap1, cmap='hot', vmin=0, vmax=1)
plt.axis('off')
plt.subplot(2, 4, 6)
# plt.title("Normalised Blur Map")
plt.imshow(blurmap2, cmap='hot', vmin=0, vmax=1)
plt.axis('off')
plt.subplot(2, 4, 7)
# plt.title("Normalised Blur Map")
plt.imshow(blurmap3, cmap='hot', vmin=0, vmax=1)
plt.axis('off')
plt.subplot(2, 4, 8)
# plt.title("Normalised Blur Map")
plt.imshow(blurmap4, cmap='hot', vmin=0, vmax=1)
plt.axis('off')


plt.tight_layout()
plt.show()
