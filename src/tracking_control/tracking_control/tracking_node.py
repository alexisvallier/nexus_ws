import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np
import math

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

def euler_from_quaternion(q):
    w=q[0]
    x=q[1]
    y=q[2]
    z=q[3]
    # euler from quaternion
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return [roll,pitch,yaw]

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.get_logger().info('Tracking Node Started')
    
        self.start_pose = None

        self.robot_world_x = None
        self.robot_world_y = None
        self.robot_world_z = None
        self.robot_world_R = None

        self.goal_x = 1.0
        self.goal_y = 1.0

        self.repel_x = 0.0
        self.repel_y = 0.0

        self.attract_x = 0.0
        self.attract_y = 0.0

        self.k_a = 2.0
        self.k_r = 0.01

        # State for control
        self.state = "TEST"

        ## New Stuff ################
        self.test_mode = True
        self.test_target = None
        self.test_start = None
        # Simple P gain
        self.kp_test = 0.3
        
        # ROS parameters
        self.declare_parameter('world_frame_id', 'odom')

        # Create a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for the control command
        self.pub_control_cmd = self.create_publisher(Twist, '/track_cmd_vel', 10)

        # subscribe to lidar detections
        self.sub_detected_objects = self.create_subscription(PoseArray, '/detected_lidar_obj', self.detected_objects_pose_callback, 10)

        # Create timer, running at 100Hz
        self.timer = self.create_timer(0.01, self.timer_update)
    
    def detected_objects_pose_callback(self, msg):
        self.get_logger().info('Received Detected LiDAR Object Pose')

        # calculate the repulsive force from the detected objects
        # for each pose, add x and y components to the repulsive force
        self.repel_x = 0.0
        self.repel_y = 0.0

        for pose in msg.poses:
            # convert pose to numpy array
            obj_pose = np.array([pose.position.x, pose.position.y, pose.position.z])
            # calculate repulsive force ( simple inverse square law )
            dist = np.linalg.norm(obj_pose)
            if dist < 0.5 or dist > 3.0:
                continue
            force_magnitude = 1.0 / (dist**2)
            force_direction = -obj_pose / dist
            force = force_magnitude * force_direction
            self.repel_x += self.k_r * force[0]
            self.repel_y += self.k_r * force[1]
        
    # not sure whether need this funciton anymore
    def get_current_poses(self):
        
        odom_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        
        # Get the current robot pose
        try:
            # from base_footprint to odom
            transform = self.tf_buffer.lookup_transform(odom_id, 'base_footprint', rclpy.time.Time())
            self.robot_world_x = transform.transform.translation.x
            self.robot_world_y = transform.transform.translation.y
            self.robot_world_z = transform.transform.translation.z
            self.robot_world_R = q2R([transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z])
        
        except TransformException as e:
            self.get_logger().error('Transform error: ' + str(e))
            return
        
        return [self.robot_world_x, self.robot_world_y, self.robot_world_z, self.robot_world_R]
    
    def timer_update(self):
        ## New Stuff
        if self.test_mode:
            self.state = "TEST"
        
        # Get the current object pose in the robot base_footprint frame
        poses = self.get_current_poses()

        if poses is None:
            return

        #reached goal, stop the robot
        if np.linalg.norm([poses[0] - self.goal_x, poses[1] - self.goal_y]) < 0.2:
            cmd_vel = Twist()
            self.pub_control_cmd.publish(cmd_vel)

        # set starting position
        if self.start_pose is None:
            self.start_pose = np.array([
                poses[0],
                poses[1]
            ])

        # calculate the attractive potential from the goal
        goal_dist = np.linalg.norm(np.array([poses[0], poses[1]]) - np.array([self.goal_x, self.goal_y]))
        goal_angle = np.arctan2(self.goal_y - poses[1], self.goal_x - poses[0])

        attractive_field = 0.5*(goal_dist**2)
        self.attract_x = self.k_a * attractive_field * np.cos(goal_angle)
        self.attract_y = self.k_a * attractive_field * np.sin(goal_angle)

        self.get_logger().info(f'####################################')
        self.get_logger().info(f'Angle: {goal_angle}')
        self.get_logger().info(f'Attractive Force: ({self.attract_x:.2f}, {self.attract_y:.2f})')
        self.get_logger().info(f'Repulsive Force: ({self.repel_x:.2f}, {self.repel_y:.2f})')
        self.get_logger().info(f'Current Pose: ({poses[0]:.2f}, {poses[1]:.2f})')

        cmd_vel = self.controller()
        self.get_logger().info(f'Control Command: ({cmd_vel.linear.x:.2f}, {cmd_vel.linear.y:.2f}, {cmd_vel.angular.z:.2f})')
        # publish the control command
        self.pub_control_cmd.publish(cmd_vel)
    
    def controller(self):
        cmd_vel = Twist()

        # potential field
        cmd_vel.linear.x = self.attract_x + self.repel_x
        cmd_vel.linear.y = self.attract_y + self.repel_y
        cmd_vel.angular.z = 0.0

        # Saturation
        cmd_vel.linear.x = max(min(cmd_vel.linear.x, 0.5),-0.5)
        cmd_vel.linear.y = max(min(cmd_vel.linear.y, 0.5),-0.5)
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, 1.2), -1.2)
        
        return cmd_vel

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    tracking_node = TrackingNode()
    rclpy.spin(tracking_node)
    # Destroy the node explicitly
    tracking_node.destroy_node()
    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
