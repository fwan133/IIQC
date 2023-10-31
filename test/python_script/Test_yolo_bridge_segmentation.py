from ultralytics import YOLO
import cv2

# Load a pretrained YOLOv8n model
model = YOLO('/home/feng/Code/catkin_ros/src/IIQC/config/bridge_seg_m.pt')

# Load the image
img = cv2.imread('/home/feng/Code/catkin_ros/src/IIQC/data/images/train_003.JPG')

results = model(img)
annotated_frame = results[0].plot()
cv2.imwrite('Bridge_Mask.jpg', annotated_frame)

cv2.imshow("bridge", annotated_frame)
cv2.waitKey()
cv2.destroyAllWindows()
