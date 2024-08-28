import rclpy
from rclpy.node import Node
import rclpy.qos

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from sklearn.linear_model import RANSACRegressor
import random
from scipy.optimize import least_squares


import time
import threading

class Turtlebot3Follow(Node):

    def __init__(self):
        super().__init__('turtlebot3_follow_node')

        # definition of publisher and subscriber object to /cmd_vel and /scan 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)

        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.ranges = []
        # list of distance in angles range
        self.range_view = []
        # list of rays angles in angles range
        self.angle_view = []
        self.stop = False
        timer_period = 0.1  # seconds

        self.timer = self.create_timer(timer_period, self.control_loop)


    # loop each 0.1 seconds
    def control_loop(self):
        msg_pub = Twist()
        msg_pub.linear.x  = self.linear_vel
        msg_pub.angular.z = self.angular_vel
        self.publisher_.publish(msg_pub)

    def stop_robot(self):
        self.stop = True
    
    

        
    # called each time new message is published
    def laser_callback(self, msg):


        def detect_shape1(ranges, angles):
            # Example RANSAC implementation for plane (cube) and circle (sphere) detection
            # RANSAC for detecting a plane (cube sides)
            
            
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            points = np.vstack((x, y)).T
            print(x)
            print()
            print(y)
            print()
            print(points)
            
            ransac_plane = RANSACRegressor()
            ransac_plane.fit(points[:, 0].reshape(-1, 1), points[:, 1])
            plane_inliers = ransac_plane.inlier_mask_
            plane_outliers = np.logical_not(plane_inliers)
    

            # RANSAC for detecting a circle (sphere)
            def circle_model(params, points):
                xc, yc, r = params
                x, y = points[:, 0], points[:, 1]
                return (x - xc) * 2 + (y - yc) * 2 - r ** 2

            def circle_fit1(points):
                # Initial guess for circle parameters
                x_m = np.mean(points[:, 0])
                y_m = np.mean(points[:, 1])
                r_m = np.mean(np.sqrt(np.abs(points[:, 0] - x_m) * 2 + np.abs(points[:, 1] - y_m) * 2))
                return [x_m, y_m, r_m]

            ransac_circle = RANSACRegressor()
            ransac_circle.fit(points, circle_fit1(points))
            circle_inliers = ransac_circle.inlier_mask_
            circle_outliers = np.logical_not(circle_inliers)

            if np.sum(plane_inliers) > np.sum(circle_inliers):
                return 'cube', points[plane_inliers]
            else:
                return 'sphere', points[circle_inliers]
            
        

        
        def detect_shape(ranges, angles):

            
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            points = np.vstack((x, y)).T


            def fit_circle(points):
            
                def circle_residuals(params, points):
                    xc, yc, r = params
                    x, y = points[:, 0], points[:, 1]
                    return np.sqrt((x - xc) ** 2 + (y - yc) ** 2) - r #**2

                x_m = np.mean(points[:, 0])
                y_m = np.mean(points[:, 1])
                r_m = np.mean(np.sqrt((points[:, 0] - x_m) ** 2 + (points[:, 1] - y_m) ** 2))
                initial_guess = [x_m, y_m, r_m]
                result = least_squares(circle_residuals, initial_guess, args=(points,))
                return result.x

            def get_circle_inliers(points, circle_params, threshold=0.1):
                xc, yc, r = circle_params
                x, y = points[:, 0], points[:, 1]
                distances = np.sqrt((x - xc) ** 2 + (y - yc) ** 2)
                inliers = points[np.abs(distances - r) < threshold]
                return inliers
    

            # RANSAC for detecting a plane (cube sides)
            ransac_plane = RANSACRegressor()
            ransac_plane.fit(points[:, 0].reshape(-1, 1), points[:, 1])
            plane_inliers = ransac_plane.inlier_mask_

            # Custom RANSAC for detecting a circle (sphere)
            best_circle_params = None
            best_circle_inliers = []

            for _ in range(100):  # Number of RANSAC iterations
                sample_indices = random.sample(range(points.shape[0]), 3)
                sample_points = points[sample_indices]
                circle_params = fit_circle(sample_points)
                circle_inliers = get_circle_inliers(points, circle_params)
                #print(f"normale:{circle_inliers}, best:{best_circle_inliers} ")
                if len(circle_inliers) > len(best_circle_inliers):
                    best_circle_params = circle_params
                    best_circle_inliers = circle_inliers

            
            #print(f"cube:{sum(plane_inliers)}, sphere:{len(best_circle_inliers)} ")
            if sum(plane_inliers) > len(best_circle_inliers):
                return 'cube'#, points[plane_inliers]
            else:
                return 'sphere'#, points[best_circle_inliers]

        


        # stop robot when shutdown node
        if self.stop:
            self.linear_vel = 0.0
            self.angular_vel = 0.0

            msg_pub = Twist()
            msg_pub.linear.x  = 0.0
            msg_pub.angular.z = 0.0
            self.publisher_.publish(msg_pub)
            return
        
        self.range_view.clear()
        self.angle_view.clear()
        
        # check sanity of ranges array
        #if self.len_ranges < 100:
        #    self.linear_vel = 0.0
        #    self.angular_vel = 0.0
        #    return

        if self.linear_vel > 0.22:
            self.linear_vel = 0.0
        if self.angular_vel > 2.8 or self.angular_vel < -2.8:
            self.angular_vel = 0.0  
        
        # if msg.range_min < 0.2:
        #     self.linear_vel = 0
        #     self.angular_vel = 0
        
        # if msg.range_max > 0.6:
        #     self.linear_vel = 0
        #     self.angular_vel = 0
        # Write your code here
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.ranges = msg.ranges
        
        

        self.ranges = self.ranges[1:46] + self.ranges[-45:]  
        angles = np.linspace(-np.pi/4, np.pi/4, len(self.ranges))      
        #good_distance = np.where(self.ranges > 0.2 and self.ranges<0.6, self.ranges, 1000)
        good_distance = []
        right_distance = 0.0
        for i, el in enumerate(self.ranges):
            if el>0.2 and el<0.6:
                
                good_distance.append([el, i]) 
        if good_distance == []:
            self.linear_vel = 0.0
            self.angular_vel = 0.0
        else:
            right_distance, idx_right_dist = min(good_distance)
            shape = detect_shape(self.ranges, angles)
            print(shape)
            if idx_right_dist > 46:
                angle = self.angle_min + (idx_right_dist*self.angle_increment) - 6.28
            else:
                angle = self.angle_min + (idx_right_dist*self.angle_increment)

            self.linear_vel = 0.05 #right_distance**2
            self.angular_vel = 0.05 #(angle**2)*np.sign(angle)
            #print(right_distance)
            
        if right_distance < 0.2 or right_distance>0.6:
                self.linear_vel = 0.0
                self.angular_vel = 0.0

def main(args=None):

    rclpy.init(args=args)

    turtlebot3_follow_node = Turtlebot3Follow()

    t = threading.Thread(target=rclpy.spin, args=[turtlebot3_follow_node])
    t.start()

    try:
        while rclpy.ok():
            time.sleep(5)
    except KeyboardInterrupt:
        turtlebot3_follow_node.stop_robot()
        time.sleep(0.5)

    # Destroy the node explicitly

    turtlebot3_follow_node.destroy_node()
    rclpy.shutdown()

    t.join()


if __name__ == '__main__':
    main()




    

    






        #ranges = np.array(msg.ranges)
        #angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

