import rclpy
from rclpy.node import Node

from std_msgs.msg import String
#from std_msgs.msg import Int8


class Shazam(Node):

    def __init__(self):
        super().__init__('shazam')
        self.hotspot_subscriber = self.create_subscription(
            String,
            '/moon_base_out',
            self.hotspot_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/moon_base_in',
            1
        )

        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.letmein_callback)
        self.i = 0

    
    def  hotspot_callback(self, msg):
        self.get_logger().info(f'received message: {msg.data}')
        hotspot_name, hotspot_psw = msg.data.split()
        self.get_logger().info(f'hotspot name: {hotspot_name};  hotspot password: {hotspot_psw}')


    def letmein_callback(self):
        msg = String()
        msg.data = 'leo09'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing {self.i:5}: msg: {msg.data}')
        self.i += 1

 
def main(args=None):
    rclpy.init(args=args)
 
    ali_baba = Shazam()
    rclpy.spin(ali_baba)
 
     # destroy the node when it is not used anymore
    ali_baba.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
