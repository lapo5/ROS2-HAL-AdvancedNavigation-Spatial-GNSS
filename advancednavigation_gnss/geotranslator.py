import numpy as np

import math
import time
import sys
import os
ros2_home = os.getenv('ROS2_HOME', "/home/roxy/ros2_ws")
sys.path.append(os.path.join(ros2_home, "src/AdvancedNavigation_HybridNavSystem/advancednavigation_gnss"))
from geo_to_cart_cxx.geo_to_cart_cxx import geo_to_cart
from advancednavigation_spatial import advancednavigation_spatial 

class GeoTranslator:
    def __init__(self, heading_toward_north):

        self.heading_toward_north = (heading_toward_north / 180.0 * math.pi) 
        self.rotation_matrix = np.eye(3, dtype=np.float32)
        self.rotation_matrix[0, 0] = math.cos(self.heading_toward_north)
        self.rotation_matrix[0, 1] = - math.sin(self.heading_toward_north)
        self.rotation_matrix[1, 0] = math.sin(self.heading_toward_north)
        self.rotation_matrix[1, 1] = math.cos(self.heading_toward_north)
        self.rotation_matrix[2, 2] = 1

        self.geotedic_pose = np.zeros([3, 1], dtype=np.float64)
        self.geotedic_confidences = np.zeros([3, 1], dtype=np.float64)
        self.cartesian_geo_pose = np.zeros([3, 1], dtype=np.float64)
        self.cartesian_geo_confidences = np.zeros([3, 3], dtype=np.float64)
        self.cartesian_pose = np.zeros([3, 1], dtype=np.float64)

        self.tf = np.eye(4, dtype=np.float32)
        self.cartesian_confidences = np.zeros([3, 3], dtype=np.float32)
        self.hal_gps = advancednavigation_spatial.HAL()
        self.name = self.hal_gps.get_gnss_name()
        self.geo_cart_transformer = geo_to_cart(self.geotedic_pose, self.geotedic_confidences,
                                                self.cartesian_geo_pose,
                                                self.cartesian_geo_confidences)
        self.res = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)

    def init(self):
        b = self.hal_gps.init("/dev/ttyUSB0")
        print("GNS - Waiting fix...", end='')
        while not self.hal_gps.has_fix():
            pass
        print(" Done.")
        self.hal_gps.get_gnss_info(self.res)
        self.geotedic_pose[:, 0] = self.res[0:3]
        self.geotedic_confidences[:, 0] = self.res[3:6]

        self.geo_cart_transformer.compute()
        return b

    def acquire(self):
        print("A")
        self.hal_gps.get_gnss_info(self.res)
        print("B")
        print(self.res)
        self.geotedic_pose[:, 0] = self.res[0:3]
        self.geotedic_confidences[:, 0] = self.res[3:6]

        self.geo_cart_transformer.compute()

        self.cartesian_pose = np.matmul(self.rotation_matrix, self.cartesian_geo_pose)

        self.cartesian_confidences = np.matmul(self.rotation_matrix, self.cartesian_geo_confidences)

        # Adjusting the frame
        self.tf[0, 3] = self.cartesian_pose[0, 0]
        self.tf[1, 3] = self.cartesian_pose[1, 0]
        self.tf[2, 3] = self.cartesian_pose[2, 0]

        return True

    def reset(self):
        self.hal_gps.get_gnss_info(self.res)
        self.geotedic_pose[:, 0] = self.res[0:3]
        self.geotedic_confidences[:, 0] = self.res[3:6]
        self.geo_cart_transformer.reset()

        self.cartesian_pose = self.cartesian_geo_pose
        self.cartesian_confidences = np.matmul(self.rotation_matrix, self.cartesian_geo_confidences)

        self.tf[0, 3] = 0.0
        self.tf[1, 3] = 0.0
        self.tf[2, 3] = 0.0

    def shutdown(self):
        self.hal_gps.shutdown()

    def get_starting_lat(self):
        return self.geo_cart_transformer.get_starting_lat()

    def get_starting_lon(self):
        return self.geo_cart_transformer.get_starting_lon()

    def get_starting_alt(self):
        return self.geo_cart_transformer.get_starting_alt()
