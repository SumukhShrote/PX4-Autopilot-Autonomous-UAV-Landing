import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.model = YOLO("/home/sumukh/autonomous_drone_landing_ws/src/object_detector/weights/best.pt")       # Load YOLO model   
        
        self.image_sub = rospy.Subscriber("/quad_f450_camera/camera_link/raw_image", Image, self.image_callback)
        
        self.error_pub = rospy.Publisher("/yolo/center_error", Float32MultiArray, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: {0}".format(e))
            return

        results = self.model.predict(source=cv_image, show=True, verbose=False)    # Run YOLO inference

        if results[0].boxes:
            box = results[0].boxes[0]     # Get the first detected bounding box
            x_min, y_min, x_max, y_max = box.xyxy[0]

            object_center_x = (x_min + x_max) / 2
            object_center_y = (y_min + y_max) / 2

            image_center_x = cv_image.shape[1] / 2     # Calculate image center
            image_center_y = cv_image.shape[0] / 2

            error_x = object_center_x - image_center_x    # Calculate x-axis error
            error_y = object_center_y - image_center_y    # Calculate y-axis error

            error_msg = Float32MultiArray(data=[error_x, error_y])      # Publish errors
            self.error_pub.publish(error_msg)
            # Draw detected object center
            cv2.circle(cv_image, (int(object_center_x), int(object_center_y)), 5, (0, 255, 0), -1)
            # Display error text
            text = f"Error (x, y): ({error_x:.1f}, {error_y:.1f})"
            cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        else:
            rospy.loginfo("No objects detected")     # Log if no detections

        cv2.imshow("YOLO Detection", cv_image)      # Display the frame
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = YoloDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
