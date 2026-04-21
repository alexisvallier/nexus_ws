# adapted from color_goal_detection.py
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseArray
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np
import struct
import sys

## Functions for quaternion and rotation matrix conversion
## The code is adapted from the general_robotics_toolbox package
## Code reference: https://github.com/rpiRobotics/rpi_general_robotics_toolbox_py
def hat(k):
    """
    Returns a 3 x 3 cross product matrix for a 3 x 1 vector

             [  0 -k3  k2]
     khat =  [ k3   0 -k1]
             [-k2  k1   0]

    :type    k: numpy.array
    :param   k: 3 x 1 vector
    :rtype:  numpy.array
    :return: the 3 x 3 cross product matrix
    """

    khat=np.zeros((3,3))
    khat[0,1]=-k[2]
    khat[0,2]=k[1]
    khat[1,0]=k[2]
    khat[1,2]=-k[0]
    khat[2,0]=-k[1]
    khat[2,1]=k[0]
    return khat

def q2R(q):
    """
    Converts a quaternion into a 3 x 3 rotation matrix according to the
    Euler-Rodrigues formula.
    
    :type    q: numpy.array
    :param   q: 4 x 1 vector representation of a quaternion q = [q0;qv]
    :rtype:  numpy.array
    :return: the 3x3 rotation matrix    
    """
    
    I = np.identity(3)
    qhat = hat(q[1:4])
    qhat2 = qhat.dot(qhat)
    return I + 2*q[0]*qhat + 2*qhat2
######################

class LidarObjDetectionNode(Node):
    def __init__(self):
        super().__init__('lidar_object_detection_node')
        self.get_logger().info('LiDAR Object Detection Node Started')
        
        # Declare the parameters for the color detection
        self.declare_parameter('dist_low', 0.0)
        self.declare_parameter('dist_high', 1.0)
        
        # Create a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for the detected object and the bounding box
        self.pub_detected_obj = self.create_publisher(PoseArray, '/detected_lidar_obj',10)
        # Create a subscriber to the LiDAR LaserScan topic
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

    def lidar_callback(self, points_msg):
        #self.get_logger().info('Received LiDAR Messages')
        # get ROS parameters
        param_dist_low = self.get_parameter('dist_low').value
        param_dist_high = self.get_parameter('dist_high').value
        
        # filter points by distance
        ranges = np.array(points_msg.ranges)
        angles = points_msg.angle_min + np.arange(len(ranges)) * points_msg.angle_increment
        valid_indices = np.where((ranges >= param_dist_low) & (ranges <= param_dist_high))[0]
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        # convert to Cartesian coordinates
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.vstack((x, y)).T

        # may not need to change much
        try:
            # Transform the center point from the camera frame to the world frame
            transform = self.tf_buffer.lookup_transform('base_footprint','laser_link',rclpy.time.Time(),rclpy.duration.Duration(seconds=0.2))
            t_R = q2R(np.array([transform.transform.rotation.w,transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z]))
            
            #create a pose message for the detected object
            detected_obj_pose = PoseArray()
            detected_obj_pose.header.frame_id = 'base_footprint'
            detected_obj_pose.header.stamp = points_msg.header.stamp

            # transform each point in points to the world frame and add to the pose array
            for i in range(points.shape[0]):
                point = points[i]
                point_3d = np.array([point[0], point[1], 0.0])
                cp_robot = t_R@point_3d+np.array([transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z])
                pose = PoseStamped()
                pose.pose.position.x = cp_robot[0]
                pose.pose.position.y = cp_robot[1]
                pose.pose.position.z = 0.0
                if pose.pose.position.x <= 0.0 and (pose.pose.position.y <= 0.3 and pose.pose.position.y >= -0.3):
                    continue
                else:
                    detected_obj_pose.poses.append(pose.pose)

        except TransformException as e:
            self.get_logger().error('Transform Error: {}'.format(e))
            return
        
        # Publish the detected object
        self.pub_detected_obj.publish(detected_obj_pose)
        
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    lidar_obj_detection_node = LidarObjDetectionNode()
    # Spin the node so the callback function is called.
    rclpy.spin(lidar_obj_detection_node)
    # Destroy the node explicitly
    lidar_obj_detection_node.destroy_node()
    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()

