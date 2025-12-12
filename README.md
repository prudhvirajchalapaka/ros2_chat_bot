# ROS 2 Jazzy AI Chatbot (Multimodal Gemini)

A real-time, intelligent chatbot integrated into **ROS 2 Jazzy**.

This project uses a **Topic-based architecture** (Publisher/Subscriber) to connect a custom Python Tkinter GUI Client with an AI "Brain" node powered by Google's advanced **Gemini 1.5 Pro** model.

It features conversation memory (context awareness) and multimodal capabilities, allowing it to analyze uploaded documents and images.

![Main GUI Screenshot](images/screenshot_gui_chat.png)
*The Tkinter GUI client communicating with the AI node.*

---

## Features

* **ROS 2 Jazzy Native:** Built using `rclpy` and standard ROS 2 topic patterns.
* **Advanced AI Model:** Uses Google Gemini 1.5 Pro for high-quality reasoning.
* **Conversation Memory:** The bot remembers previous context in the chat session.
* **Multimodal:** Supports uploading PDFs, text files, and images for analysis via the GUI.
* **Custom GUI:** Lightweight Python Tkinter interface separate from the bot logic.

---

## Architecture

The project consists of two main nodes communicating asynchronously over topics.

![Architecture Diagram](images/architecture_diagram.png)

1.  **`gui_client` Node:** Provides the user interface. Publishes user text and file paths to the `/user_prompt` topic and subscribes to `/bot_reply` to display answers.
2.  **`ai_bot` Node:** Subscribes to `/user_prompt`. It manages the Google Gemini API connection, handles file uploads to memory, maintains chat history, and publishes responses to the `/bot_reply` topic.

---

## Prerequisites

* **Ubuntu 24.04 (Noble Numbat)**
* **ROS 2 Jazzy Jalisco** installed and configured.
* **Python 3**
* A **Google AI Studio API Key** (Get one for free at [aistudio.google.com](https://aistudio.google.com/)).

---

## Installation & Setup

### 1. Install System Dependencies
Ensure you have the Python package installer and the Tkinter GUI library.

```bash
sudo apt update
sudo apt install python3-pip python3-tk