import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from geometry_msgs.msg import Point
import torch.backends.cudnn as cudnn
from numpy import random
from models.experimental import attempt_load
from utils.plots import plot_one_box
from utils.torch_utils import select_device, time_synchronized
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path

class YOLOv7ROS(Node):
    def __init__(self, target_object="chair"):
        super().__init__('yolov7_ros')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = attempt_load('coco.weights', map_location=self.device)  # Load model
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[255, 0, 0]] * len(self.names)  # Set color for bounding boxes
        self.img_size = 640
        self.conf_thres = 0.25
        self.iou_thres = 0.45
        self.half = self.device.type != 'cpu'
        self.target_object = target_object  # Set the target object from the argument
        self.xy_pub = self.create_publisher(Point, 'xy_coordinates', 10)  # Publisher for XY coordinates

        if self.half:
            self.model.half()  # Convert model to fp16

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # Preprocess image
        img = cv2.resize(cv_image, (self.img_size, self.img_size))
        img = img[:, :, ::-1].transpose(2, 0, 1)  # Convert BGR to RGB and transpose to [3, H, W]
        img = img.copy()  # Ensure positive strides

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # Convert to fp16/32
        img /= 255.0  # Normalize 0 - 255 to 0.0 - 1.0

        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        with torch.no_grad():
            pred = self.model(img, augment=False)[0]

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if len(det):
                # Rescale boxes from img_size to original image size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], cv_image.shape).round()

                # Print detected objects and draw bounding boxes
                for *xyxy, conf, cls in reversed(det):
                    # Replace traffic light, kite, and tv with chair
                    if self.names[int(cls)] in ["traffic light", "kite", "tv"]:
                        self.names[int(cls)] = "chair"

                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    print(f'Detected: {label} at {xyxy}')  # Print detection
                    plot_one_box(xyxy, cv_image, label=label, color=self.colors[int(cls)], line_thickness=1)

                    # Publish coordinates only for the target object (including replaced "chair")
                    if self.names[int(cls)] == self.target_object:
                        # Calculate the center of the bounding box
                        x_center = (xyxy[0] + xyxy[2]) / 2
                        y_center = (xyxy[1] + xyxy[3]) / 2

                        # Create Point message
                        point_msg = Point()
                        point_msg.x = float(x_center)
                        point_msg.y = float(y_center)
                        point_msg.z = 0.0  # z coordinate is not used in this case

                        # Publish the center coordinates
                        self.xy_pub.publish(point_msg)

        # Display the image with bounding boxes
        cv2.imshow("YOLOv7-Tiny Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    # Get the target object from the argument list (default to "chair" if not provided)
    target_object = "chair"
    if len(args) > 1:
        target_object = args[1]

    yolov7_ros = YOLOv7ROS(target_object)
    rclpy.spin(yolov7_ros)
    yolov7_ros.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)

