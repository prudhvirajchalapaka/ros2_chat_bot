# ROS 2 Jazzy AI Chatbot (Multimodal Gemini)

A real-time, intelligent chatbot integrated into **ROS 2 Jazzy**,
featuring multimodal AI powered by **Gemini 1.5 Pro** and a lightweight
Python Tkinter GUI.

This project uses a **topic-based architecture** (Publisher/Subscriber)
to connect a GUI client with an AI "Brain" node. It supports
conversation memory and analysis of uploaded documents and images.

## ✨ Features

-   **ROS 2 Jazzy Native:** Built using `rclpy` and standard ROS 2 topic
    communication.
-   **Advanced AI Model:** Integrates **Google Gemini 1.5 Pro** for
    reasoning and multimodal understanding.
-   **Conversation Memory:** Keeps chat context during the session.
-   **Multimodal Support:** PDF, TXT, and image understanding.
-   **Custom GUI:** Clean Tkinter interface independent of bot logic.

## 🏗 Architecture

The system consists of two nodes communicating asynchronously over ROS 2
topics.

1.  **gui_client Node**
    -   Publishes user prompts and file paths to `/user_prompt`
    -   Subscribes to `/bot_reply`
    -   Displays messages in Tkinter GUI
2.  **ai_bot Node**
    -   Subscribes to `/user_prompt`
    -   Connects to Google Gemini API
    -   Handles file uploads (PDF, TXT, Images)
    -   Maintains conversation memory
    -   Publishes responses to `/bot_reply`

## 📦 Prerequisites

-   Ubuntu 24.04 (Noble Numbat)
-   ROS 2 Jazzy Jalisco
-   Python 3
-   Google AI Studio API Key (https://aistudio.google.com/)

## 🚀 Installation & Setup

### 1. Install System Dependencies

    sudo apt update
    sudo apt install python3-pip python3-tk

### 2. Install Python AI Libraries

    python3 -m pip install google-generativeai python-dotenv mimetypes --break-system-packages

### 3. Create & Configure Your ROS 2 Workspace

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src

### 4. Configure Your API Key

    export GEMINI_API_KEY="YOUR_ACTUAL_LONG_API_KEY_HERE"

### 5. Build the Package

    cd ~/ros2_ws
    colcon build --packages-select ros2_chat_bot
    source install/setup.bash

## ▶️ Usage

### Terminal 1 -- Start the AI Brain

    ros2 run ros2_chat_bot ai_bot

### Terminal 2 -- Start the GUI Client

    ros2 run ros2_chat_bot gui_client

## 🖼 Multimodal File Upload

1.  Click the upload button in the GUI
2.  Select a PDF, TXT, PNG, or JPG file
3.  Ask questions about the uploaded document or image

## 📄 License (Apache License 2.0)

Apache License Version 2.0, January 2004 http://www.apache.org/licenses/
