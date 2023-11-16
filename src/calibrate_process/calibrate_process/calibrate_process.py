#!/user/bin/env python

import rclpy
import os
import message_filters
import csv
import cv2
from datetime import datetime
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Joy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import polanalyser

qos_button_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                         history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                         depth=1)
qos_vrpn_cam_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                           history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                           depth=1)

# Set this path to where you'd like to save the images
parent_path = "/Users/audibailey/repos/thesis-ws/results"

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')

        # check parent path is valid
        if not os.path.isdir(parent_path):
            self.get_logger().error("No path to save data!")
            exit(1)

        self.latest_cam_pos = None
        self.latest_sample_pos = None
        self.latest_light_pos = None
        self.latest_image = None

        self.pose_cam = self.create_subscription(PoseStamped, '/vrpn_mocap/Camera/pose', self.save_cam, qos_vrpn_cam_policy)
        self.pose_light = self.create_subscription(PoseStamped, '/vrpn_mocap/Light/pose', self.save_light, qos_vrpn_cam_policy)
        self.pose_sample = self.create_subscription(PoseStamped, '/vrpn_mocap/Sample/pose', self.save_sample, qos_vrpn_cam_policy)

        self.cam_sub = self.create_subscription(Image, '/lucid_vision/camera_1/image', self.save_image, qos_vrpn_cam_policy)
        self.ps_sub = self.create_subscription(Joy, '/ds4_driver/joy', self.process_data, qos_button_policy)

        self.br = CvBridge()

    def save_cam(self, pos):
        self.latest_cam_pos = pos

    def save_light(self, pos):
        self.latest_light_pos = pos

    def save_sample(self, pos):
        self.latest_sample_pos = pos

    def save_image(self, image):
        self.latest_image = image

    def process_data(self, controller):
        # ensure square was pressed and an image has been saved
        square = controller.buttons[0] == 1
        if not square:
            self.get_logger().info("Square not pressed...")
            return

        if not self.latest_image:
            self.get_logger().info("Camera not ready yet")
            return

        self.get_logger().info("Begin Processing...")

        # convert image binary to OpenCV image
        try:
            cv_image = self.br.imgmsg_to_cv2(self.latest_image, 'bgr8')
        except CvBridgeError as error:
            self.get_logger().error(error)
            return

        # create timestamp path
        iso_date = datetime.now().isoformat('T')  # ISO-8601 (sortable)
        new_path = os.path.join(parent_path, iso_date)
        try:
            os.mkdir(new_path)
        except OSError as error:
            self.get_logger().error("Couldn't make new path!")
            return

        # write image
        image_path = os.path.join(new_path, "image.jpeg")
        cv2.imwrite(image_path, cv_image)

        aolp, dolp = self.process_image(new_path, cv_image[:, :, 0])
        self.process_csv(self.latest_light_pos, self.latest_cam_pos, self.latest_sample_pos, iso_date, new_path, aolp, dolp)

        self.get_logger().info("Finished Processing...")

    def process_image(self, new_path, image):
        contour_details = (0, 0, 0, 0)
        ##################################################
        # attempt to see if there is calibration rectangle
        ##################################################
        # convert to grey
        img_grey = image

        # create threshold
        adaptive = cv2.adaptiveThreshold(img_grey, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 51, 9)

        # modify threshold to be dilated and eroded - this helps connect the rectangle perimeter
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
        opening = cv2.morphologyEx(adaptive, cv2.MORPH_CLOSE, kernel, iterations=5)

        # find contours
        contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.015 * peri, True)
            # looking for a single large rectangle
            if len(approx) == 4 and area >= 5000:
                self.get_logger().info("Image calibrated.")
                x, y, w, h = cv2.boundingRect(contour)
                contour_details = (x, y, x + w, y + h)

        if contour_details == (0, 0, 0, 0):
            self.get_logger().info("No contour found, image hasn't been calibrated")
            return

        # crop the image and process it
        image_cropped = image[contour_details[1]:contour_details[3], contour_details[0]:contour_details[2]]
        image_list = polanalyser.demosaicing(image_cropped, polanalyser.COLOR_PolarMono)
        angles = np.deg2rad([0, 45, 90, 135])

        img_stokes = polanalyser.calcStokes(image_list, angles)

        img_intensity = polanalyser.cvtStokesToIntensity(img_stokes)
        img_dolp = polanalyser.cvtStokesToDoLP(img_stokes)
        img_aolp = polanalyser.cvtStokesToAoLP(img_stokes)

        img_dolp_u8 = np.clip(img_dolp * 255, 0, 255).astype(np.uint8)
        img_aolp_u8 = polanalyser.applyColorToAoLP(img_aolp)

        # export
        cv2.imwrite(f"{new_path}/export_intensity.png", img_intensity)
        cv2.imwrite(f"{new_path}/export_DoLP.png", img_dolp_u8)
        cv2.imwrite(f"{new_path}/export_AoLP.png", img_aolp_u8)

        return np.mean(img_dolp_u8), np.median(img_aolp_u8)

    def process_csv(self, light, camera, sample, iso_date, new_path, aolp, dolp):
        # prepare the csv values
        light_pos = np.array([
            light.pose.position.x,
            light.pose.position.y,
            light.pose.position.z
        ])
        camera_pos = np.array([
            camera.pose.position.x,
            camera.pose.position.y,
            camera.pose.position.z
        ])
        sample_pos = np.array([
            sample.pose.position.x,
            sample.pose.position.y,
            sample.pose.position.z
        ])
        phase_info = self.calculate_angles(light_pos, camera_pos)
        pose_header = [
            'time/photo',
            'x - light',
            'y - light',
            'z - light',
            'x - cam',
            'y - cam',
            'z - cam',
            'x - sample',
            'y - sample',
            'z - sample',
            'phase',
            'zenith',
            'sampled - aolp',
            'sampled - dolp'
        ]
        pose_info = [
            iso_date,
            str(light_pos[0]),
            str(light_pos[1]),
            str(light_pos[2]),
            str(camera_pos[0]),
            str(camera_pos[1]),
            str(camera_pos[2]),
            str(sample_pos[0]),
            str(sample_pos[1]),
            str(sample_pos[2]),
            str(phase_info[0]),
            str(phase_info[1]),
            str(aolp),
            str(dolp)
        ]
        # write to the csv, either append or write from scratch
        csv_path = os.path.join(parent_path, "auto_polarisation_info.csv")
        if not os.path.isfile(csv_path):
            with open(csv_path, mode='a') as polarisation_file:
                writer = csv.writer(polarisation_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(pose_header)
                writer.writerow(pose_info)
        else:
            with open(csv_path, mode='a') as polarisation_file:
                writer = csv.writer(polarisation_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(pose_info)

    def calculate_angles(self, rel_light, camera_location):
        phase_angle = self.calc_phase(camera_location, rel_light)
        vector_plane_normal = np.cross(camera_location, rel_light)  # calculate normal vector to plane
        vector_surface_normal = (0, 0, 1)  # normal vector to surface which azmith is calculated from
        zenith = np.subtract(self.calc_phase(vector_surface_normal, vector_plane_normal), 90)  # zenith angle of plane
        return phase_angle, zenith

    def cart2spherical(self, vector3d):
        x = vector3d[0]
        y = vector3d[1]
        z = vector3d[2]

        # calculate dependants
        rho = np.sqrt(np.sum(np.square([x, y, z])))
        theta_rad = np.arctan2(y, x)
        phi_rad = np.arctan2(np.sqrt(np.sum(np.square([x, y]))), z)

        # convert radians to degrees
        theta = np.rad2deg(theta_rad)
        phi = np.rad2deg(phi_rad)

        # round values
        rho = np.round(rho, 0)
        theta = np.round(theta, 2)
        phi = np.round(phi, 2)

        return rho, theta, phi

    def calc_phase(self, vector1, vector2):
        # Calculate the dot product of the two vectors
        dot_product = np.dot(vector1, vector2)

        # Calculate the magnitudes of the vectors
        magnitude1 = np.linalg.norm(vector1)
        magnitude2 = np.linalg.norm(vector2)

        # Calculate the cosine of the angle using the dot product and magnitudes
        cosine_angle = np.divide(dot_product, np.multiply(magnitude1, magnitude2))

        # Calculate the angle in radians using the arccosine of the cosine
        angle_radians = np.arccos(cosine_angle)

        # Convert the angle from radians to degrees
        angle_degrees = np.degrees(angle_radians)

        # Round
        angle_degrees = np.round(angle_degrees, 2)

        return angle_degrees

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()