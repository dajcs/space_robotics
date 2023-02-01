import rclpy
from rclpy.node import Node

# ros2 topic info /random_encrypted_msg
# Type: std_msgs/msg/String
# std_msgs/msg -> import from
# /String ->
from std_msgs.msg import String


class HotSpotSpy(Node):
 
    def __init__(self):
        super().__init__('hot_spot_spy')
        self.my_subscriber = self.create_subscription(
            String,
            '/moon_base_out',
            self.moonbase_callback,
            10)

        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)
        self.i = 0


        self.publisher = self.create_publisher(String, '/moon_base_in', 1)  # 1 - queue length



    def publish_callback(self):
        msg = String()
        msg.data = 'leo09'
        self.publisher.publish(msg)


    def moonbase_callback(self, msg):
	# ros2 interface show std_msgs/msg/String 
	# string data  -> msg has one field of type str

        self.get_logger().info(f'received message: {msg.data}')
        leoid, ssid, psw = msg.data.split()
        print(f'\n leo_id: {leoid}\n SSID: {ssid}\n psw: {psw}\n\n')

 
def main(args=None):
    rclpy.init(args=args)
 
    ali_baba = HotSpotSpy()
    rclpy.spin(ali_baba)
 
     # destroy the node when it is not used anymore
    ali_baba.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
