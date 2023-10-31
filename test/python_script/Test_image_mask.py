import cv2
import numpy as np

# Step 1: Load the image and define the polygon points
image_path = '../../data/CapturedData/HastingsBridge/Pier1/1_colour.jpg'
image = cv2.imread(image_path)

height, width = image.shape[:2]
# Define the points of the polygon (in this example, a triangle)
polygon_points = np.array([(100, 100), (300, 50), (200, 250)], np.int32)

# Step 2: Create a mask with the same dimensions as the image
mask = np.zeros((height, width), dtype=np.uint8)

# Step 3: Fill the polygon with white color in the mask
cv2.fillPoly(mask, [polygon_points], 255)

# Step 4: Bitwise AND the mask with the original image to get the region of interest (ROI)
masked_image = cv2.bitwise_and(image, image, mask=mask)

# Step 5: Create a copy of the original image
segmented_image = image.copy()

# Step 6: Overlay the masked region on the segmented image
segmented_image[mask > 0] = [0, 255, 0]  # Set the masked region to green (you can choose any color)

# Display the results
cv2.imshow('Original Image', image)
cv2.imshow('Masked Image', masked_image)
cv2.imshow('Segmented Image', segmented_image)
cv2.waitKey(0)
cv2.destroyAllWindows()