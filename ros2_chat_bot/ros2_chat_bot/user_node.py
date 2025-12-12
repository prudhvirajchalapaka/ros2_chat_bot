import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class UserNode(Node):
    def __init__(self):
        super().__init__('user_node')
        
        # Publisher: Sends our prompt to the bot
        self.publisher_ = self.create_publisher(String, 'user_prompt', 10)
        
        # Subscriber: Listens for the bot's reply
        self.subscription = self.create_subscription(
            String,
            'bot_reply',
            self.bot_reply_callback,
            10)
        
        self.get_logger().info('User Node Started. Type something and press Enter!')
        
        # Run input in a separate thread so it doesn't block the ROS callbacks
        input_thread = threading.Thread(target=self.get_user_input)
        input_thread.daemon = True
        input_thread.start()

    def get_user_input(self):
        while rclpy.ok():
            try:
                # Get input from console
                text = input("You > ")
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
            except EOFError:
                break

    def bot_reply_callback(self, msg):
        # Print the bot's reply clearly
        print(f"\n🤖 Bot > {msg.data}\nYou > ", end="", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = UserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()