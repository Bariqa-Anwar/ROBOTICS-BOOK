---
sidebar_position: 2
---

# LLM for Command Understanding and Response
<!-- DIAGRAM: HRI_Workflow -->

Once speech has been converted to text using a Speech-to-Text (STT) model like Whisper, the next step in achieving natural Human-Robot Interaction (HRI) is to interpret these text commands and generate an appropriate response or action. Large Language Models (LLMs) like OpenAI's GPT series are exceptionally good at understanding natural language, extracting intent, and generating coherent text, making them ideal for this role.

This section will demonstrate how to integrate an LLM (specifically GPT) into a ROS 2 node to parse human commands, generate a natural language response, and integrate with the robot's action system.

## 1. Setting up the Python Environment for OpenAI API (GPT)

As with Whisper, you will use the OpenAI API. Ensure your `OPENAI_API_KEY` environment variable is set and the `openai` Python library is installed in your ROS 2 virtual environment.

```bash
# Activate your ROS 2 Python virtual environment first
# source ~/ros2_ws/venv/bin/activate
pip install openai
```

## 2. Creating a `command_parser_node.py`

This ROS 2 node will subscribe to the transcribed text from the `speech_to_text_node.py`, send it to the GPT API, interpret the command, and formulate a response.

```python
# ros2_ws/src/my_robot_pkg/nodes/command_parser_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os

class CommandParserNode(Node):
    def __init__(self):
        super().__init__('command_parser_node')
        self.subscription = self.create_subscription(
            String,
            '/human_speech_text', # Subscribe to transcribed speech
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.robot_response_publisher = self.create_publisher(String, '/robot_response_text', 10)
        self.openai_client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

        self.get_logger().info("Command Parser Node initialized. Waiting for commands...")

    def listener_callback(self, msg: String):
        human_command = msg.data
        self.get_logger().info(f'Received command: "{human_command}"')

        # Use GPT to understand command and generate response
        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo", # Or "gpt-4"
                messages=[
                    {"role": "system", "content": "You are a helpful robot assistant. Respond concisely."},
                    {"role": "user", "content": human_command}
                ]
            )
            robot_response = response.choices[0].message.content
            self.get_logger().info(f'GPT responded: "{robot_response}"')

            # Publish the robot's natural language response
            response_msg = String()
            response_msg.data = robot_response
            self.robot_response_publisher.publish(response_msg)

        except Exception as e:
            self.get_logger().error(f"Error calling OpenAI GPT API: {e}")
            response_msg = String()
            response_msg.data = "I'm sorry, I could not process that command."
            self.robot_response_publisher.publish(response_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommandParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Creating a `text_to_speech_node.py`

To complete the auditory interaction loop, the robot's text response needs to be converted back into speech. A simple Text-to-Speech (TTS) node can achieve this.

```python
# ros2_ws/src/my_robot_pkg/nodes/text_to_speech_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# import gtts # Google Text-to-Speech
# from playsound import playsound # For playing audio - requires platform-specific setup
# import os

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')
        self.subscription = self.create_subscription(
            String,
            '/robot_response_text', # Subscribe to robot's text responses
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.get_logger().info("Text-to-Speech Node initialized. Waiting for robot responses...")

    def listener_callback(self, msg: String):
        robot_text = msg.data
        self.get_logger().info(f'Received robot response: "{robot_text}"')

        try:
            # Convert text to speech (conceptual)
            # For a real implementation, you would use a TTS library like gTTS
            # tts = gtts.gTTS(robot_text, lang='en')
            # audio_file = "robot_response.mp3"
            # tts.save(audio_file)
            # playsound(audio_file)
            # os.remove(audio_file) # Clean up audio file

            self.get_logger().info(f'Simulating speech for: "{robot_text}"')
        except Exception as e:
            self.get_logger().error(f"Error during text-to-speech conversion: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
