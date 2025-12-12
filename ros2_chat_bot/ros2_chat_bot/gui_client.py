import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import tkinter as tk
from tkinter import scrolledtext, Entry, Button, END

class ChatClientNode(Node):
    def __init__(self, gui_app):
        super().__init__('gui_client_node')
        self.gui = gui_app
        
        self.publisher_ = self.create_publisher(String, 'user_prompt', 10)
        self.subscription = self.create_subscription(
            String, 'bot_reply', self.bot_reply_callback, 10)

    def bot_reply_callback(self, msg):
        # 🟢 Thread-Safe Call: Queue the GUI update to the main thread
        self.gui.root.after(0, 
                            self.gui.display_message, 
                            f"🤖 Bot: {msg.data}", 
                            "bot")

    def send_to_ros(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)

class ChatApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS 2 AI Chatbot")
        self.root.geometry("500x600")

        # 1. Chat Display Area (Scrollable)
        self.chat_area = scrolledtext.ScrolledText(root, state='disabled', wrap=tk.WORD)
        self.chat_area.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        
        self.chat_area.tag_config("user", foreground="blue")
        self.chat_area.tag_config("bot", foreground="green")

        # 2. Input Frame
        input_frame = tk.Frame(root)
        input_frame.pack(padx=10, pady=10, fill=tk.X)

        self.entry_field = Entry(input_frame, font=("Arial", 12))
        self.entry_field.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
        self.entry_field.bind("<Return>", self.send_message)

        send_btn = Button(input_frame, text="Send", command=self.send_message, bg="#0084ff", fg="white")
        send_btn.pack(side=tk.RIGHT)

        # 3. Initialize ROS in a separate thread
        self.ros_node = None # Initialize attribute
        self.start_ros()

    def start_ros(self):
        # rclpy.init is usually done in main, but fine here if only called once.
        if not rclpy.ok():
             rclpy.init() 
        self.ros_node = ChatClientNode(self)
        
        # Run ROS spin in a separate thread so GUI doesn't freeze
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()

    def send_message(self, event=None):
        text = self.entry_field.get()
        if text.strip() != "":
            # This is called by the main thread (GUI event)
            self.display_message(f"You: {text}", "user")
            
            # This sends data to the ROS Thread, which is fine
            self.ros_node.send_to_ros(text)
            
            self.entry_field.delete(0, END)

    def display_message(self, message, sender_tag):
        # This function is now always called by the main thread
        self.chat_area.configure(state='normal')
        self.chat_area.insert(tk.END, message + "\n\n", sender_tag)
        self.chat_area.configure(state='disabled')
        self.chat_area.yview(tk.END)

    def on_close(self):
        # Ensure we shut down ROS cleanly before destroying the GUI
        if rclpy.ok():
            self.ros_node.destroy_node()
            rclpy.shutdown()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = ChatApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == '__main__':
    main()