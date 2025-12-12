import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import google.generativeai as genai
import os

class AIBotNode(Node):
    def __init__(self):
        super().__init__('ai_bot_node')
        
        # 1. Get the API Key from the environment
        api_key = os.getenv('GEMINI_API_KEY')
        if not api_key:
            self.get_logger().error("❌ No API Key found! Did you run 'export GEMINI_API_KEY=...'?")
        else:
            genai.configure(api_key=api_key)
            self.model = genai.GenerativeModel('gemini-2.5-flash-lite') # Using 'Flash' model for speed
            self.get_logger().info("✅ Gemini AI Connected Successfully!")

        # Subscriber & Publisher
        self.subscription = self.create_subscription(
            String, 'user_prompt', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(String, 'bot_reply', 10)
        
        self.get_logger().info('🤖 AI Bot is initializing... Ready to chat!')

    def listener_callback(self, msg):
        user_text = msg.data
        self.get_logger().info(f'Received: "{user_text}"')
        
        response_text = "I am offline (No API Key)."

        # 2. Call the Real AI
        try:
            if hasattr(self, 'model'):
                # Generate content
                response = self.model.generate_content(user_text)
                response_text = response.text
            else:
                response_text = "Error: API Key missing. Please set GEMINI_API_KEY."

        except Exception as e:
            self.get_logger().error(f"API Error: {str(e)}")
            response_text = "I encountered an error connecting to the AI brain."

        # 3. Publish the response
        reply_msg = String()
        reply_msg.data = response_text
        self.publisher_.publish(reply_msg)
        self.get_logger().info(f'Replied: "{response_text[:50]}..."')

def main(args=None):
    rclpy.init(args=args)
    node = AIBotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()