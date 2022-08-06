import sys
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import time


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


def capture_point_cloud_xyz(camera: Camera, i: int):
    img, depth_img = camera.capture_rgbd_images()

    np.save(f'images/img{i}.npy', img)
    np.save(f'depth_images/img{i}.npy', depth_img)

    depth_img[depth_img < 1300] = 0

    rgb_d_img = o3d.geometry.RGBDImage.create_from_color_and_depth(img, depth_img)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgb_d_img)

    o3d.visualization.draw_geometries([pcd], width=640, height=540)
    o3d.io.write_point_cloud(f'point clouds/img{i}.ply', pcd)


def main(delay: float):
    camera = Camera()
    camera.pipeline.start(camera.config)

    time.sleep(delay)

    n = 0
    running = True
    while running:
        capture_point_cloud_xyz(camera, n)
        n += 1

    camera.pipeline.stop()


if __name__ == "__main__":
    delay_ = float(sys.argv[1])

    print(f"Starting... waiting for {delay_} seconds...")
    main(delay_)





