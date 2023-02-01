import rclpy
from rclpy.node import Node

'''
subscribing to leo0x/firmware/imu

leo package on foxy:
sudo apt-get install ros-foxy-leo

leo package on humble:
sudo apt install ros-humble-leo-msgs

ros2 topic list -> /leo06/firmware/imu

ros2 topic echo /leo06/firmware/imu --> to monitor messages
stamp:
  sec: 1665149630
  nanosec: 642894081
temperature: 34.35990524291992
gyro_x: -0.02250371128320694
gyro_y: -0.00905474741011858
gyro_z: -0.010918959975242615
accel_x: -0.8978256583213806
accel_y: 0.24420857429504395
accel_z: 9.513360023498535


ros2 topic info /leo06/firmware/imu
Type: leo_msgs/msg/Imu
Publisher count: 1
Subscription count: 0

ros2 interface show leo_msgs/msg/Imu
# This message holds the data retrieved from an Accel/Gyro+Temp IMU sensor
#
# The temperature field represents the temperature reported by the sensor in Degrees Celcius
# The gyro_* fields represent the rotational velocity in rad/s
# The accel_* fields represent the linear acceleration in m/s^2

builtin_interfaces/Time stamp
	int32 sec
	uint32 nanosec
float32 temperature
float32 gyro_x
float32 gyro_y
float32 gyro_z
float32 accel_x
float32 accel_y
float32 accel_z


accel_x: 6  (set polygon)
ros2 topic pub /leo06/firmware/imu leo_msgs/Imu "{stamp: {sec: 1, nanosec: 1}, temperature: 1, gyro_x: 0, gyro_y: 0, gyro_z: 0, accel_x: 6, accel_y: 0, accel_z: 10}" -r 10

accel_y: 6  (set_speed)
ros2 topic pub /leo06/firmware/imu leo_msgs/Imu "{stamp: {sec: 1, nanosec: 1}, temperature: 1, gyro_x: 0, gyro_y: 0, gyro_z: 0, accel_x: 0, accel_y: 6, accel_z: 10}" -r 10

accel_z: -10 (ctrl-C)
ros2 topic pub /leo06/firmware/imu leo_msgs/Imu "{stamp: {sec: 1, nanosec: 1}, temperature: 1, gyro_x: 0, gyro_y: 0, gyro_z: 0, accel_x: 0, accel_y: 0, accel_z: -10}" -r 10


'''

'''
publishing on leo0x/cmd_vel

ros2 topic info /leo06/cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 0

ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z

    "leo01": "Earth leorover",
    "leo05": "Mercury curiosity",
    "leo06": "Venus perseverance",
    "leo07": "Mars sojourner",
    "leo08": "Jupiter lunokhod",
    "leo09": "Saturn opportunity",
    "leo10": "Pluto ingenuity"


WheelOdom.get_fields_and_field_types()                                                                                                         

{'stamp': 'builtin_interfaces/Time',
 'velocity_lin': 'float',
 'velocity_ang': 'float',
 'pose_x': 'float',
 'pose_y': 'float',
 'pose_yaw': 'float'}


'''

from leo_msgs.msg import Imu
from leo_msgs.msg import WheelOdom

from math import pi
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

LINEAR_SPEED = 0.2    # [m/s]
RECTANGLE_X = 2       # [m]
RECTANGLE_Y = 1       # [m]
PRINT_PERIOD = 100    # print every 100th message 

x_time = RECTANGLE_X / LINEAR_SPEED   # [s] 10
y_time = RECTANGLE_Y / LINEAR_SPEED   # [s] 5

MY_LEO = 'leo09'

qos_profile = QoSProfile(
    reliability = QoSReliabilityPolicy.BEST_EFFORT,
    history = QoSHistoryPolicy.KEEP_LAST,
    depth = 1
)


class LeoRover(Node):
    def __init__(self):
        super().__init__('patrol')

        self.imu_subscriber = self.create_subscription(
            Imu,
            f'/{MY_LEO}/firmware/imu',
            self.imu_msg_callback,
            qos_profile = qos_profile
        )
        self.odometer_subscriber = self.create_subscription(
            WheelOdom,
            f'/{MY_LEO}/firmware/odometer',                  # ???????? - TODO
            self.odometer_msg_callback,
            qos_profile = qos_profile
        )

        self.leo_publisher = self.create_publisher(Twist, f'/{MY_LEO}/cmd_vel', 1)

        self.turtle_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)   # monitoring cmd_vel on turtle_sim

        self.i = -1                        # counter
        self.linear_speed = LINEAR_SPEED   # initial linear speed


    def init_state(self, msg):
            self.t0 = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            self.theta = 0
            self.turn = False
            self.rect_x = True
            self.latitude_time = x_time
            self.drive_time = 0

            return


    def print(self, msg):
        self.get_logger().info(f'''
        ---------------------------------------------------------------
        stamp.sec, stamp.nanosec::  {msg.stamp.sec}.{msg.stamp.nanosec}
        ---------------------------------------------------------------
        accel_xyz: {round(msg.accel_x,2):5} {round(msg.accel_y,2):5} {round(msg.accel_z,2):5} 
        gyro_xyz:  {round(msg.gyro_x,2):5} {round(msg.gyro_y,2):5} {round(msg.gyro_z,2):5} 

        {self.operation}\n\n''')


    def odometer_msg_callback(self, msg):
#        self.get_logger().info(msg)
        pass

    def imu_msg_callback(self, msg):
        '''
        receiving imu message below
            stamp:
                sec: 1665149630
                nanosec: 642894081
            temperature: 34.35990524291992
            gyro_x: -0.02250371128320694
            gyro_y: -0.00905474741011858
            gyro_z: -0.010918959975242615
            accel_x: -0.8978256583213806
            accel_y: 0.24420857429504395
            accel_z: 9.513360023498535
        '''
        self.i += 1 # step counter
        if not self.i: # initialize state - couldn't be done in __init__ because we need timestamp for t0
            self.init_state(msg)
            return
        else:
            t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            dt = t - self.t0
            self.t0 = t


        cmd_vel = Twist()

        if self.turn == False:                  # go ahead
            cmd_vel.linear.x = self.linear_speed
            self.leo_publisher.publish(cmd_vel)
            self.turtle_publisher.publish(cmd_vel)   # can be observed on the turtlesim
            cmd = f'Linear drive, speed = {self.linear_speed} m/s,  rectangle X latitude: {self.rect_x})'
            self.drive_time += dt
            if self.drive_time > self.latitude_time:    # linear.x driving time limited by latitude_time
                self.drive_time = 0
                self.turn = True
        
        else:                                   # turning time
            cmd_vel.angular.z = 0.5
            self.leo_publisher.publish(cmd_vel)
            cmd = 'Turn left (angular.z = 0.5)'
            cmd_vel.angular.z = msg.gyro_z
            self.turtle_publisher.publish(cmd_vel)     # following on turtle
            d_theta = msg.gyro_z * dt
            self.theta += d_theta
            if self.theta > pi / 2:
                self.theta = 0
                self.turn = False
                self.rect_x = not self.rect_x      # alternating rect_x and rect_y
                self.latitude_time = x_time if self.rect_x else y_time
        
        self.operation = f'''Executing: {cmd}
        State: drive_time: {round(self.drive_time,2):5}    theta: {round(self.theta,2):5}'''

        if PRINT_PERIOD:
            if self.i % PRINT_PERIOD == 0:    # print only every PRINT_PERIOD time
                self.print(msg)



def main(args=None):
    rclpy.init(args=args)

    leo_rover = LeoRover()
    rclpy.spin(leo_rover)

    # after spin is over - i.e. Ctrl-C has been pressed
    leo_rover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

