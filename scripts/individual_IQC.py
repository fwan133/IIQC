import cv2
import numpy as np
from ultralytics import YOLO
import matplotlib.pyplot as plt


class IndividualIQC:
    def __init__(self, _id, image):
        self.id = _id
        self.image = image
        self.grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.pose6d = []
        self.ROI = None
        self.blur_map = None
        self.exposure_map = None
        self.SSD = None

    def predictROI(self, yolo_model):
        results = yolo_model.predict(source=self.image, stream=False)
        self.ROI = np.array(results[0].masks.xy).astype(int)

    def printROI(self):
        print(self.ROI)

    def showROI(self):
        if self.ROI is None:
            print('Warning: Please predict the ROI before showing it')
        else:
            edge_color = (0, 0, 255)
            fill_color = (0, 0, 255)
            copied_image = self.image.copy()
            cv2.polylines(copied_image, self.ROI, isClosed=True, color=edge_color, thickness=2)
            mask = cv2.fillPoly(np.zeros(self.image.shape, dtype=np.uint8), self.ROI, color=fill_color)
            mask_img = cv2.addWeighted(copied_image, 1, mask, 0.5, 0)
            cv2.imshow('Image with ROI', mask_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def estimateBlurMap(self, kernel_size):
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Calculate the Laplacian variance using the specified kernel size
        laplacian_var = cv2.Laplacian(gray_image, cv2.CV_64F, ksize=kernel_size).var()

        # Normalize the variance to the range [0, 1]
        normalized_var = laplacian_var / 10000.0  # You can adjust the normalization factor as needed

        # Create the blur map by replicating the normalized variance to each channel
        self.blur_map = np.ones(image.shape[:2]) * normalized_var

    def estimateExposureMap(self):
        print('The mean gray level of an image is: ', np.mean(self.grey_image))
        # Calculate the histogram of the original image
        hist = cv2.calcHist([self.image], [0], None, [256], [0, 266])
        print("Original: ", np.mean(hist))
        # Create a binary image according to the ROI
        mask = np.zeros(self.image.shape[:2], dtype=np.uint8)
        binary_mask = cv2.fillPoly(mask, self.ROI, 255)
        image_masked = cv2.bitwise_and(self.image, self.image, mask=binary_mask)
        hist_mask = cv2.calcHist([image_masked], [0], mask, [256], [0, 256])
        print("Bridge part: ", np.mean(hist_mask))
        # Plot the results
        fig, axs = plt.subplots(2, 2, figsize=(12, 12))
        axs[0, 0].imshow(cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB), cmap='gray')
        axs[0, 0].axis('off')
        axs[0, 1].imshow(cv2.cvtColor(image_masked, cv2.COLOR_BGR2RGB), cmap='gray')
        axs[0, 1].axis('off')
        axs[1, 0].bar(range(256), hist.ravel(), color='black')
        axs[1, 0].set_xlim([0, 256])
        axs[1, 1].plot(hist_mask, color='black')
        axs[1, 1].set_xlim([0, 256])

        plt.show()

        # # Calculate the mean value of grey scale
        # grey_image = self.grey_image
        # area_of_interest = cv2.bitwise_and(grey_image, grey_image, mask=binary_mask)
        # cv2.imshow("grey", area_of_interest)
        # cv2.waitKey()
        #
        # print("Mean Grayscale Value:", np.mean(area_of_interest))


def is_blurry(image, x, y, w, h, threshold=100):
    # Crop the specified area from the image
    area_of_interest = image[y:y + h, x:x + w]

    # Convert the area to grayscale
    gray_area = cv2.cvtColor(area_of_interest, cv2.COLOR_BGR2GRAY)

    # Compute the Laplacian variance as a measure of image sharpness
    laplacian_var = cv2.Laplacian(gray_area, cv2.CV_64F).var()
    print(laplacian_var)

    # Check if the area is blurry based on the threshold
    return laplacian_var < threshold


def mouse_callback(event, x, y, flags, param):
    global clicked_x, clicked_y

    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_x, clicked_y = x, y
        print("Clicked at (x, y):", x, y)


if __name__ == '__main__':
    # Create an IndividualIQC object
    image = cv2.imread('../data/images/train_001.JPG')
    individual_IQC = IndividualIQC(_id='1', image=image)

    # Check the bridge segmentation
    model = YOLO("../config/bridge_seg_m.pt")
    individual_IQC.predictROI(model)
    # individual_IQC.showROI()

    # Check the exposure intensity
    individual_IQC.estimateExposureMap()

    # Check the blurry
    # cv2.namedWindow("Image", cv2.WINDOW_FULLSCREEN)
    # individual_IQC.estimateBlurMap(5)
    # cv2.imshow("Image", individual_IQC.image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
