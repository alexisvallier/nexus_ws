import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
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
        
        # Current object pose
        self.obs_pose = None
        self.goal_pose = None
        # Initialize start pose for return
        self.start_pose = None
        self.robot_world_x = None
        self.robot_world_y = None
        self.robot_world_z = None
        self.robot_world_R = None

        # State for control
        self.state = "GOAL"
        
        # ROS parameters
        self.declare_parameter('world_frame_id', 'odom')

        # Create a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for the control command
        self.pub_control_cmd = self.create_publisher(Twist, '/track_cmd_vel', 10)
        # Create a subscriber to the detected object pose
        self.sub_detected_goal_pose = self.create_subscription(PoseStamped, 'detected_color_object_pose', self.detected_obs_pose_callback, 10)
        self.sub_detected_obs_pose = self.create_subscription(PoseStamped, 'detected_color_goal_pose', self.detected_goal_pose_callback, 10)

        # Create timer, running at 100Hz
        self.timer = self.create_timer(0.01, self.timer_update)
    
    def detected_obs_pose_callback(self, msg):
        self.get_logger().info('Received Detected Object Pose')
        
        odom_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        center_points = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        # TODO: Filtering
        # You can decide to filter the detected object pose here
        # For example, you can filter the pose based on the distance from the camera
        # or the height of the object
        if np.linalg.norm(center_points) > 3.0 or center_points[2] > 0.7:
            return
        
        try:
            # Transform the center point from the camera frame to the world frame
            transform = self.tf_buffer.lookup_transform(odom_id,msg.header.frame_id,rclpy.time.Time(),rclpy.duration.Duration(seconds=0.1))
            t_R = q2R(np.array([transform.transform.rotation.w,transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z]))
            cp_world = t_R@center_points+np.array([transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z])
        except TransformException as e:
            self.get_logger().error('Transform Error: {}'.format(e))
            return
        
        # Get the detected object pose in the world frame
        self.obs_pose = cp_world

    def detected_goal_pose_callback(self, msg):
        self.get_logger().info('Received Detected Object Pose')
        
        odom_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        center_points = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        # TODO: Filtering
        # You can decide to filter the detected object pose here
        # For example, you can filter the pose based on the distance from the camera
        # or the height of the object
        if np.linalg.norm(center_points) > 3.0 or center_points[2] > 0.7:
            return
        
        try:
            # Transform the center point from the camera frame to the world frame
            transform = self.tf_buffer.lookup_transform(odom_id,msg.header.frame_id,rclpy.time.Time(),rclpy.duration.Duration(seconds=0.1))
            t_R = q2R(np.array([transform.transform.rotation.w,transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z]))
            cp_world = t_R@center_points+np.array([transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z])
        except TransformException as e:
            self.get_logger().error('Transform Error: {}'.format(e))
            return
        
        # Get the detected object pose in the world frame
        self.goal_pose = cp_world
        
    def get_current_poses(self):
        
        odom_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        # Get the current robot pose
        try:
            # from base_footprint to odom
            transform = self.tf_buffer.lookup_transform('base_footprint', odom_id, rclpy.time.Time())
            self.robot_world_x = transform.transform.translation.x
            self.robot_world_y = transform.transform.translation.y
            self.robot_world_z = transform.transform.translation.z
            self.robot_world_R = q2R([transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z])

            obstacle_pose = self.robot_world_R@self.obs_pose+np.array([self.robot_world_x,self.robot_world_y,self.robot_world_z])
            goal_pose = self.robot_world_R@self.goal_pose+np.array([self.robot_world_x,self.robot_world_y,self.robot_world_z])
        
        except TransformException as e:
            self.get_logger().error('Transform error: ' + str(e))
            return
        
        return obstacle_pose, goal_pose
    
    def timer_update(self):
        ################### Write your code here ###################
        
        # Now, the robot stops if the object is not detected
        # But, you may want to think about what to do in this case
        # and update the command velocity accordingly
        if (self.state == "GOAL") and (self.goal_pose is None):
            cmd_vel = Twist() 
            # spin to try to find goal again
            cmd_vel.angular.z = 0.1
            self.pub_control_cmd.publish(cmd_vel)
            return
        
        # Get the current object pose in the robot base_footprint frame
        current_obs_pose, current_goal_pose = self.get_current_poses()
        
        # TODO: get the control velocity command
        cmd_vel = self.controller()
        
        # publish the control command
        self.pub_control_cmd.publish(cmd_vel)
        #################################################
    
    def controller(self):
        # Instructions: You can implement your own control algorithm here
        # feel free to modify the code structure, add more parameters, more input variables for the function, etc.
        
        ########### Write your code here ###########
        cmd_vel = Twist()

        # set starting position
        if self.start_pose is None:
            self.start_pose = np.array([
                self.robot_world_x,
                self.robot_world_y
            ])

        # Get poses
        obs_pose, goal_pose = self.get_current_poses()

        # Goal position in robot frame
        gx = goal_pose[0]
        gy = goal_pose[1]

        # Distance + angle to goal
        goal_dist = np.sqrt(gx**2 + gy**2)
        angle = np.arctan2(gy, gx)

        # Obstacle Position
        ox, oy = None, None
        obs_dist = None
        
        if obs_pose is not None:
            ox = obs_pose[0]
            oy = obs_pose[1]
            obs_dist = np.sqrt(ox**2 + oy**2)

        # Gain
        k_lin = 0.5
        k_ang = 1.0
        
        # State machine ( bug 0 )
        
        # moving toward goal
        if self.state == "GOAL":
            self.get_logger().info('STATE: Goal')
            if obs_pose is not None:
                obs_angle = np.arctan2(oy,ox)

                if abs(obs_angle) < 0.4 and obs_dist < 0.8:
                    # Decide direction once
                    self.state = "AVOID"

            # goal reached
            if goal_dist < 0.4:
                self.state = "RETURN"
        
            # Goal Tracking
            cmd_vel.angular.z = k_ang * angle
            forward_gain = max(0.2, 1.0 - abs(angle))
            cmd_vel.linear.x = k_lin * goal_dist * forward_gain

        # obstacle encountered
        elif self.state == "AVOID":
            self.get_logger().info('STATE: Avoid')
            # obstacle avoided
            if obs_pose is None:
                self.state = "GOAL"
                return Twist()

            obs_angle = np.arctan2(oy, ox)
            des_theta = obs_angle + np.pi/2

            gain = 0.8 - obs_dist

            # move perpendicular to obstacle
            cmd_vel.linear.x = 0.8 * gain * np.cos(des_theta)
            cmd_vel.linear.y = 0.8 * gain * np.sin(des_theta)
            cmd_vel.angular.z = 0.0

            if abs(np.arctan2(oy, ox)) > 0.8 or obs_dist > 1.0:
                self.state = "GOAL"

        elif self.state == "RETURN":
            self.get_logger().info('STATE: Return')
            # need to do opposite of current pose (go to robot frame origin)
            # need to change this, without a goal pose in the camera it won't work
            # reassign gx and gy to the robot's position from the starting point

            #### New code ####
            rx = self.robot_world_x
            ry = self.robot_world_y
            dx = self.start_pose[0] - rx
            dy = self.start_pose[1] - ry

            R_wr = self.robot_world_R
            R_rw = R_wr.T                 

            error_world = np.array([dx, dy, 0])
            error_robot = R_rw @ error_world

            dx_r = error_robot[0]
            dy_r = error_robot[1]
            
            home_dist = np.sqrt(dx_r**2 + dy_r**2)
            home_angle = np.arctan2(dy_r, dx_r)
            ###################

            # avoid obstacle if needed
            if obs_pose is not None:
                obs_angle = np.arctan2(oy, ox)
                if abs(obs_angle) < 0.4 and obs_dist < 0.8:
                    self.state = "AVOIDR"
    
            if home_dist < 0.3:
                self.state = "DONE"
                return Twist()

            # Goal Tracking
            cmd_vel.angular.z = k_ang * home_angle
            forward_gain = max(0.2, 1.0 - abs(home_angle))
            cmd_vel.linear.x = k_lin * home_dist * forward_gain

        elif self.state == "AVOIDR":
            self.get_logger().info('STATE: Avoiding on Return')
            # obstacle avoided
            if obs_pose is None:
                self.state = "RETURN"
                return Twist()
            else:
                obs_angle = np.arctan2(oy,ox)
                
            des_theta = obs_angle + np.pi/2

            gain = 0.8 - obs_dist

            # move perpendicular to obstacle
            cmd_vel.linear.x = 0.8 * gain * np.cos(des_theta)
            cmd_vel.linear.y = 0.8 * gain * np.sin(des_theta)
            cmd_vel.angular.z = 0.0
            
            if abs(np.arctan2(oy, ox)) > 0.8 or obs_dist > 1.0:
                self.state = "RETURN"
        
        elif self.state == "DONE":
            self.get_logger().info('STATE: Done')
            return Twist()
            
        # Saturation
        cmd_vel.linear.x = min(cmd_vel.linear.x, 0.5)
        cmd_vel.linear.y = min(cmd_vel.linear.y, 0.5)
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, 1.2), -1.2)
        
        return cmd_vel
    
        ############################################

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
