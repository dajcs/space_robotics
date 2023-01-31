import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int8


class MyComplexDecryptor(Node):

    def __init__(self):
        super().__init__('my_complex_decryptor')
        self.msg_subscriber = self.create_subscription(
            String,
            '/encrypted_msg',
            self.msg_callback,
            10
        )
        self.key_subscriber = self.create_subscription(
            Int8,
            '/key',
            self.key_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            '/decrypted_msg',
            1
        )
        self.key = None

    
    def key_callback(self, msg):
        if self.key != msg.data:
            self.get_logger().info(f'>>>>>>>>>>>>>>>>>>>>>>>>>>>>> old key: {self.key} ---> new key: {msg.data} <<<<<')
            self.key = msg.data
    

    def  msg_callback(self, msg):
        self.get_logger().info(f'received encrypted message: {msg.data}')
        if self.key is not None:
            self.clear_text = self.decipher(msg.data, self.key)
            self.get_logger().info(f'the decrypted message: {self.clear_text}')

        if self.key is not None:
            my_message = String()
            my_message.data = f'Attila {self.clear_text}'
            self.publisher.publish(my_message)

    def decipher(self,text,s):
        result = ""
        # transverse the plain text
        for i in range(len(text)):
            char = text[i]
            # Encrypt uppercase characters in plain text
            
            if (char.isupper()):
                result += chr((ord(char) - s-65) % 26 + 65)
            # Encrypt lowercase characters in plain text
            else:
                result += chr((ord(char) - s-97) % 26 + 97)
        return result
 
def main(args=None):
    rclpy.init(args=args)
 
    my_complex_decryptor = MyComplexDecryptor()
    rclpy.spin(my_complex_decryptor)
 
     # destroy the node when it is not used anymore
    my_complex_decryptor.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
