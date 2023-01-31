'''
    Playing with Leo rover - moving it in different polygons
    polygons can be swithced by roll-, speed can be changed by pitching the rover, 
    and Ctrl+C is turning upside down 
'''


from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from math import pi
from leo_msgs.msg import Imu
import rclpy
from rclpy.node import Node

'''
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
'''


MY_LEO = 'leo09'
POLYGON = 2          # starting with a line
PRINT_PERIOD = 20    # print every n_th message, if 0 do not print, if 1 print all (100 Hz)
LINEAR_TIME = 10     # sum of the linear driving times during one cycle
MIN_POLYGON = 2      # min nr of latitudes - actually 2 is a line, going forth and back
MAX_POLYGON = 6      # max nr of latitudes
INITIAL_SPEED = 0.2   # initial speed
MIN_LIN_SPEED = 0.1  # minimum linear speed
MAX_LIN_SPEED = 0.4  # max linear spped  - actually max linear speed is ~0.35 m/s


pause = 40*[0.0]
alt_increase = 20*[0.0] + 20*[+1.0]
alt_decrease = 20*[0.0] + 20*[-1.0]
alt_inc_limit = 20*[+1.0] + 20*[-1.0]
alt_dec_limit = 20*[-1.0] + 20*[+1.0]


qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


class LeoRover(Node):
    def __init__(self):
        super().__init__('Speedy_Gonzales')

        self.imu_subscriber = self.create_subscription(
            Imu,
            f'/{MY_LEO}/firmware/imu',
            self.imu_msg_callback,
            qos_profile=qos_profile
        )

        self.leo_publisher = self.create_publisher(Twist, f'/{MY_LEO}/cmd_vel', 1)

        self.turtle_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)   # monitoring cmd_vel on turtle_sim

        self.i = -1                        # counter
        self.polygon = POLYGON             # initial number of polygon sides
        self.linear_speed = INITIAL_SPEED  # initial linear speed


    def init_state(self, msg):
        self.t0 = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.theta = 0
        self.turn = False
        self.drive_time = 0
        self.latitude_time = LINEAR_TIME / self.polygon
        self.feedback_signal = []
        return

    def print(self, msg):
        self.get_logger().info(f'''
        ---------------------------------------------------------------
        stamp.sec, stamp.nanosec::  {msg.stamp.sec}.{msg.stamp.nanosec}
        ---------------------------------------------------------------
        accel_xyz: {round(msg.accel_x,2):5} {round(msg.accel_y,2):5} {round(msg.accel_z,2):5} 
        gyro_xyz:  {round(msg.gyro_x,2):5} {round(msg.gyro_y,2):5} {round(msg.gyro_z,2):5} 

        {self.operation}\n\n''')

    def feedback(self, msg):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.feedback_signal.pop(0)
        self.leo_publisher.publish(cmd_vel)

        if self.feedback_signal:
            if len(self.feedback_signal) % 100 == 0:
                self.operation = f'feedback: <<<<<<<<<<<<<<< {len(self.feedback_signal)} >>>>>>>>>>> remaining'
                self.print(msg)
        else:
            self.operation = 'feedback over - init - ready for a new polygon ############################################################'
            self.init_state(msg)    # init - ready for a new polygon
            self.print(msg)

    def set_polygon(self, msg):
        if msg.accel_x > 0:
            self.polygon += 1        # increase nr sides
            self.feedback_signal = pause + alt_increase * self.polygon + pause * 4
            if self.polygon > MAX_POLYGON:
                self.polygon = MAX_POLYGON
                self.feedback_signal = pause + alt_inc_limit * self.polygon + pause * 4

        else:
            self.polygon -= 1     # decrease nr sides
            self.feedback_signal = pause + alt_decrease * self.polygon + pause * 4
            if self.polygon < MIN_POLYGON:
                self.polygon = MIN_POLYGON
                self.feedback_signal = pause + alt_dec_limit * self.polygon + pause * 4

        self.operation = f'>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   New polygon set to: {self.polygon}'
        self.print(msg)

    def set_speed(self, msg):
        if msg.accel_y > 0:
            self.linear_speed += 0.1        # increase linear speed
            self.feedback_signal = pause + alt_increase * \
                int(10*self.linear_speed) + pause * 4
            if self.linear_speed > MAX_LIN_SPEED:
                self.linear_speed = MAX_LIN_SPEED
                self.feedback_signal = pause + alt_inc_limit * \
                    int(10*self.linear_speed) + pause * 4

        else:
            self.linear_speed -= 0.1        # decrease linear speed
            self.feedback_signal = pause + alt_decrease * \
                int(10*self.linear_speed) + pause * 4
            if self.linear_speed < MIN_LIN_SPEED:
                self.linear_speed = MIN_LIN_SPEED
                self.feedback_signal = pause + alt_dec_limit * \
                    int(10*self.linear_speed) + pause * 4

        self.operation = f'>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   New Seed Limit set to: {self.linear_speed}'
        self.print(msg)

    def imu_msg_callback(self, msg):
        '''
        receiving imu message
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
        self.i += 1  # step counter
        if not self.i:  # initialize state - couldn't be done in __init__ because we need timestamp for t0
            self.init_state(msg)
            return
        else:
            t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            dt = t - self.t0
            self.t0 = t


        if msg.accel_z < 0:            # ctrl-C - time to go home :-)
            self.operation = '''
            Upside down ~ Ctrl-C pressed\n
            Thank you for using Speedy Gonzales\n\n'''
            self.print(msg)
            self.destroy_node()


        if self.feedback_signal:           # give a feedback about current polygon/speed setting
            self.feedback(msg)
            return

        if abs(msg.accel_x) > 5:            # polygon setting in progress
            self.set_polygon(msg)           # proceed with increase/decrease of polygon sides
            return

        if abs(msg.accel_y) > 5:            # speed setting in progress
            self.set_speed(msg)             # proceed with increase/decrease of speed
            return

        cmd_vel = Twist()

        if self.turn == False:                  # go ahead
            cmd_vel.linear.x = self.linear_speed
            self.leo_publisher.publish(cmd_vel)
            # can be observed on the turtlesim
            self.turtle_publisher.publish(cmd_vel)
            cmd = f'Full steam ahead (linear.x = {self.linear_speed})'
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
            if self.theta > 2 * pi / self.polygon:
                self.theta = 0
                self.turn = False

        self.operation = f'''Executing Polygon: {self.polygon}
        Publishing: {cmd}
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
