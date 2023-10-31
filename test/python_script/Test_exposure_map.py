import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
img = cv2.imread('/home/feng/Code/catkin_ros/src/IIQC/data/test/exposure/DJI_0014.JPG')

# Convert the image to float32 to avoid overflow
img_float = img.astype(np.float32)

# Calculate perceived luminance (brightness)
luminance = 0.299 * img_float[:, :, 2] + 0.587 * img_float[:, :, 1] + 0.114 * img_float[:, :, 0]

# Filter out the pixels that have luminance values between 30 and 230
luminance_filtered = np.where((luminance >= 30) & (luminance <= 230), luminance, 0)

# Normalize the luminance map to 0-255 for whole image
luminance_map = ((luminance - np.min(luminance)) / (np.max(luminance) - np.min(luminance)) * 255).astype(np.uint8)
luminance_map_filtered = ((luminance_filtered - np.min(luminance_filtered)) / (
            np.max(luminance_filtered) - np.min(luminance_filtered)) * 255).astype(np.uint8)

# Define patch size
patch_size = 17

# Calculate the number of patches in x and y directions
nx = img.shape[1] // patch_size
ny = img.shape[0] // patch_size

# Initialize exposure map
exposure_map = np.zeros((ny, nx))
exposure_map_filtered = np.zeros((ny, nx))

# Loop over the patches
for i in range(ny):
    for j in range(nx):
        # Extract the patch
        patch = luminance[i * patch_size:(i + 1) * patch_size, j * patch_size:(j + 1) * patch_size]
        patch_filtered = luminance_filtered[i * patch_size:(i + 1) * patch_size, j * patch_size:(j + 1) * patch_size]

        # Calculate the mean luminance of the patch
        exposure_map[i, j] = np.mean(patch)
        exposure_map_filtered[i, j] = np.mean(patch_filtered)

# Normalize the exposure map to 0-255
exposure_map = ((exposure_map - np.min(exposure_map)) / (np.max(exposure_map) - np.min(exposure_map)) * 255).astype(
    np.uint8)
exposure_map_filtered = ((exposure_map_filtered - np.min(exposure_map_filtered)) / (
            np.max(exposure_map_filtered) - np.min(exposure_map_filtered)) * 255).astype(np.uint8)

# Create figure and axes
fig, axs = plt.subplots(2, 4, figsize=(20, 6))

# Display the original image
axs[0, 0].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
axs[0, 0].set_title('Original Image')
axs[0, 0].axis('off')

# Display the grayscale image
axs[0, 1].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cmap='gray')
axs[0, 1].set_title('Grayscale Image')
axs[0, 1].axis('off')

# Display the luminance map for whole image
axs[0, 2].imshow(luminance_map, cmap='gray')
axs[0, 2].set_title('Luminance Map')
axs[0, 2].axis('off')

# Display the exposure map
axs[0, 3].imshow(exposure_map, cmap='gray')
axs[0, 3].set_title('Exposure Map')
axs[0, 3].axis('off')

# Display the filtered images
axs[1, 0].imshow(luminance_map_filtered, cmap='gray')
axs[1, 0].set_title('Filtered Grayscale Image')
axs[1, 0].axis('off')

# Display the grayscale image
axs[1, 1].imshow(luminance_map_filtered, cmap='gray')
axs[1, 1].set_title('Filtered Grayscale Image')
axs[1, 1].axis('off')

# Display the luminance map for whole image
axs[1, 2].imshow(luminance_map_filtered, cmap='gray')
axs[1, 2].set_title('Filtered Luminance Map')
axs[1, 2].axis('off')

# Display the exposure map
axs[1, 3].imshow(exposure_map_filtered, cmap='gray')
axs[1, 3].set_title('Filtered Exposure Map')
axs[1, 3].axis('off')

plt.tight_layout()
plt.show()
