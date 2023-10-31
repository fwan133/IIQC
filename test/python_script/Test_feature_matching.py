import cv2

# Load the images
image1 = cv2.imread('../../data/CapturedData/HastingsBridge/Pier1/1_colour.jpg', cv2.IMREAD_GRAYSCALE)
image2 = cv2.imread('../../data/CapturedData/HastingsBridge/Pier1/2_colour.jpg', cv2.IMREAD_GRAYSCALE)

# Initialize the feature detector (e.g., ORB, SIFT, SURF)
feature_detector = cv2.ORB_create()

# Detect and compute keypoints and descriptors
keypoints1, descriptors1 = feature_detector.detectAndCompute(image1, None)
keypoints2, descriptors2 = feature_detector.detectAndCompute(image2, None)

# Initialize the feature matcher (e.g., Brute-Force, FLANN)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Match descriptors between images
matches = matcher.match(descriptors1, descriptors2)

# Sort matches by distance
matches = sorted(matches, key=lambda x: x.distance)

# Draw top N matches
N = 50
matched_image = cv2.drawMatches(image1, keypoints1, image2, keypoints2, matches[:N], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

# Display the matched image
cv2.imshow('Matched Image', matched_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
