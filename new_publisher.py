import rclpy
from rclpy.node import Node
import cv2
import io
import numpy as np
import pyrealsense2 as rs

import requests
from PIL import Image
from requests_toolbelt.multipart.encoder import MultipartEncoder


def find_highest_confidence(keys):
    look_for = keys[0]
    for key in keys:
        if key['confidence'] > look_for['confidence']:
            look_for = key
    return look_for


from std_msgs.msg import Float64, Float32MultiArray

class Camera:
    def __init__(self):
        """ Set up the camera for collecting point cloud information

        :return: The pipeline, configuration, and sensor
        """
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        found_rgb = False
        for device_sensor in device.sensors:
            if device_sensor.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break

        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        sensor = pipeline_profile.get_device().query_sensors()[0]

        sensor.set_option(rs.option.laser_power, 100)
        sensor.set_option(rs.option.confidence_threshold, 3)
        sensor.set_option(rs.option.min_distance, 150)
        sensor.set_option(rs.option.enable_max_usable_range, 0)
        sensor.set_option(rs.option.receiver_gain, 18)
        sensor.set_option(rs.option.post_processing_sharpening, 3)
        sensor.set_option(rs.option.pre_processing_sharpening, 5)
        sensor.set_option(rs.option.noise_filtering, 6)

        self.config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

        device_product_line = str(device.get_info(rs.camera_info.product_line))
        # Configure the pipeline to stream different resolutions of color and depth streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # config depth stream
        if device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        align_to = rs.stream.color  # align to color stream
        self.align = rs.align(align_to)

    def capture_rgbd_images(self):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()

        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        return np.asanyarray(color_frame.get_data()), np.asanyarray(aligned_depth_frame.get_data())
    

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float64, 'topic', 10)
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'topic2', 10)

    def run(self):
        camera = Camera()
        
        transformatation_matrix = np.loadtxt('camera2base_matrix_for_image_new0.txt')

        while True:
            rgb_img, depth_image = camera.capture_rgbd_images()
            rgb_img = cv2.flip(rgb_img, 1)

            image = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
            pilImage = Image.fromarray(image)

            # Convert to JPEG Buffer
            buffered = io.BytesIO()
            pilImage.save(buffered, quality=100, format="JPEG")

            # Build multipart form and post request
            m = MultipartEncoder(fields={'file': ("imageToUpload", buffered.getvalue(), "image/jpeg")})

            response = requests.post("https://detect.roboflow.com/dataset1-m15bx/1?api_key=ZjVJrn29KikPeyvZDf38", data=m, headers={'Content-Type': m.content_type})

            keys = response.json()['predictions']
            print(keys)

            look_for = {}
            if len(keys) > 1:
                look_for = find_highest_confidence()
            elif len(keys) > 0:
                look_for = keys[0]

            if look_for != {}:
                pt1 = (
                    int(look_for['x'] - look_for['width'] / 2),
                    int(look_for['y'] - look_for['height'] / 2)
                )

                pt2 = (
                    int(look_for['x'] + look_for['width'] / 2),
                    int(look_for['y'] + look_for['height'] / 2)
                )
                
                vec = np.array([look_for['x'], look_for['y'], 1, 1])
                new_vec = np.matmul(transformatation_matrix, vec)[:2]

                center_z = depth_image[int(look_for['y'])][int(look_for['x'])]

                cv2.rectangle(rgb_img, pt1, pt2, (255, 255, 255), thickness=5)

                cv2.putText(rgb_img, str(center_z), (50, 50), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=3, color=(0, 255, 0), thickness=3)

                msg = Float64()
                msg.data = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: {msg.data}')

                msg2 = Float32MultiArray()
                msg2.data = [float(new_vec[0]), float(new_vec[1]), float(center_z)]
                self.pos_publisher.publish(msg2)
                self.get_logger().info(f'Publishing: {msg2.data}')

            # Display result frame
            cv2.imshow("frame", rgb_img)

            key = cv2.waitKey(1)
            if key == 27: # [esc] key
                break


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()
    camera_publisher.run()

    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()