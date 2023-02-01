'''
ros2 interface show leo_msgs/msg/WheelOdom
# This message represents the pose and velocity of a differential wheeled robot, 
# estimated from the wheel encoders.
#
# The velocity_* fields represent the linear and angular velocity of the robot.
# The pose_* fields represent the x, y and yaw pose of the robot w.r.t. the starting pose.
#
# The coordinate frame that represents the robot is located at the center of rotation.

builtin_interfaces/Time stamp
float32 velocity_lin
float32 velocity_ang
float32 pose_x
float32 pose_y
float32 pose_yaw

stamp:
  sec: 1665152368
  nanosec: 683160648
velocity_lin: 0.0
velocity_ang: 0.0
pose_x: 0.1369149535894394
pose_y: -0.00023565816809423268
pose_yaw: 0.00815676711499691

rate: 20 Hz
'''

import rclpy
from rclpy.node import Node

from leo_msgs.msg import Imu
from leo_msgs.msg import WheelOdom

from math import pi
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


MY_LEO = 'leo09'

qos_profile = QoSProfile(
    reliability = QoSReliabilityPolicy.BEST_EFFORT,
    history = QoSHistoryPolicy.KEEP_LAST,
    depth = 1
)


class LeoRover(Node):
    def __init__(self):
        super().__init__('patrol')

        # self.imu_subscriber = self.create_subscription(
        #     Imu,
        #     f'/{MY_LEO}/firmware/imu',
        #     self.imu_msg_callback,
        #     qos_profile = qos_profile
        # )
        self.odometer_subscriber = self.create_subscription(
            WheelOdom,
            f'/{MY_LEO}/firmware/wheel_odom',   
            self.odometer_msg_callback,
            qos_profile = qos_profile
        )

        self.leo_publisher = self.create_publisher(Twist, f'/{MY_LEO}/cmd_vel', 1)

        self.turtle_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)   # monitoring cmd_vel on turtle_sim

        self.i = -1                        # counter




    def odometer_msg_callback(self, msg):
        if self.i % 10 == 0:
            self.get_logger().info(f'''sec: {msg.stamp.sec}   nanosec: {msg.stamp.nanosec}
            velocity_lin: {msg.velocity_lin}
            velocity_ang: {msg.velocity_ang}
            pose_x: {msg.pose_x}
            pose_y: {msg.pose_y}
            pose_yaw: {msg.pose_yaw}
            ''')
        self.i += 1
        if self.i == 0:
            self.x0 = msg.pose_x
            self.y0 = msg.pose_y
            self.yaw0 = msg.pose_yaw
            self.latitude_X = True
            self.turn = False
            return

        cmd_vel = Twist()

        if self.turn:     # turning
            cmd_vel.angular.z = 0.5
            self.leo_publisher.publish(cmd_vel)
            self.turtle_publisher.publish(cmd_vel)
            yaw0 = self.yaw0
            yaw1 = msg.pose_yaw
            theta = yaw1-yaw0 if yaw1-yaw0 > 0 else yaw1-yaw0 + 2*pi
            print(f'{self.i} turning, yaw0: {yaw0}  yaw1: {yaw1} theta: {theta}\n')
            if theta > pi/2:   # next phase
                self.turn = False
                self.x0 = msg.pose_x
                self.y0 = msg.pose_y

        else:               # linear driving
            cmd_vel.linear.x = 0.5
            self.leo_publisher.publish(cmd_vel)
            self.turtle_publisher.publish(cmd_vel)
            x0 = self.x0
            y0 = self.y0
            x1 = msg.pose_x
            y1 = msg.pose_y
            d = ((x1-x0)**2 + (y1-y0)**2)**0.5
            limit = 0.5 + 0.5 * self.latitude_X
            print(f'{self.i} driving, d: {d},  limit: {limit} \n')
            if d > limit:
                self.turn = True
                self.yaw0 = msg.pose_yaw
                self.latitude_X = not self.latitude_X




    # def imu_msg_callback(self, msg):
    #     self.get_logger().info(msg)
    #     pass




def main(args=None):
    rclpy.init(args=args)

    leo_rover = LeoRover()
    rclpy.spin(leo_rover)

    # after spin is over - i.e. Ctrl-C has been pressed
    leo_rover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

