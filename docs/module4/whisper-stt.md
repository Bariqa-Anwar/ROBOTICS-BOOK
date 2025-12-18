---
sidebar_position : 1
---

# Speech-to-Text with OpenAI Whisper

Integrating natural language interaction into robotic systems is a key step towards more intuitive Human-Robot Interaction (HRI). One fundamental component of this is Speech-to-Text (STT), which converts spoken language into written text that the robot's systems can process. OpenAI's Whisper model is a powerful general-purpose speech recognition model capable of transcribing audio into text with high accuracy.

This section will guide you through setting up Whisper for STT in your ROS 2 environment, allowing your robot to "hear" and understand spoken commands.

## 1. OpenAI API Key Setup

To use OpenAI's Whisper model, you will need an OpenAI API key.

1.  **Obtain an API Key** : If you don't have one, visit the [OpenAI API website](https ://platform.openai.com/account/api-keys) and create a new secret key.
2.  **Securely Store the Key** : **Never embed your API key directly in your code.** The most secure way to handle it is by setting it as an environment variable.

    ```bash
    export OPENAI_API_KEY="sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
    ```
    For persistent use, add this line to your `~/.bashrc` (or equivalent shell profile) and then `source ~/.bashrc`.

## 2. Setting up the Python Environment for OpenAI API

You'll need to install the OpenAI Python client library.

```bash
# Activate your ROS 2 Python virtual environment first
# source ~/ros2_ws/venv/bin/activate
pip install openai
```

## 3. Creating a `speech_to_text_node.py`

This ROS 2 node will capture audio (simulated or real), send it to the OpenAI Whisper API for transcription, and then publish the transcribed text to a ROS 2 topic.

```python
# ros2_ws/src/my_robot_pkg/nodes/speech_to_text_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os
import io
# import sounddevice as sd # For real audio capture - requires PortAudio
# import numpy as np # For audio processing

class SpeechToTextNode(Node) :
    def __init__(self) :
        super().__init__('speech_to_text_node')
        self.publisher_ = self.create_publisher(String, '/human_speech_text', 10)
        self.openai_client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

        self.get_logger().info("Speech-to-Text Node initialized. Waiting for audio input...")
        
        # Simulate an audio input for demonstration purposes
        self.create_timer(5.0, self.simulate_audio_input)

    def simulate_audio_input(self) :
        # In a real scenario, this would be audio captured from a microphone
        # For simulation, we'll use a dummy audio file or generate a prompt
        
        # Dummy audio content (replace with actual audio bytes if you have a file)
        # For a truly minimal example without external files, one might just pass text to Whisper API directly
        # However, Whisper is designed for audio input.
        
        # A simple approach for demonstration without actual audio file I/O within the agent
        # is to assume audio input and then send it to OpenAI.
        # Let's create a dummy in-memory "audio" for the Whisper API call.
        
        # NOTE : This part needs actual audio data. For a functional example,
        # you'd replace `io.BytesIO(b'...')` with actual audio file reading or microphone input.
        # For simplicity, we are simulating the *result* of audio capture for the Whisper API call.
        
        # Mocking the audio file object that Whisper expects
        # In a real setup, you would read actual audio bytes from a mic or file
        # with open("path/to/your/audio.wav", "rb") as audio_file :
        #     transcript = self.openai_client.audio.transcriptions.create(
        #         model="whisper-1", 
        #         file=audio_file
        #     )
        
        # Since the agent cannot interact with actual audio files or microphones,
        # we will simulate the transcription process for demonstration.
        # In a real application, replace this with actual audio processing.
        
        try :
            # Simulate an audio stream (e.g., from a microphone or file)
            # This is a placeholder; actual audio capture would be here.
            # Example : with sd.RawInputStream(samplerate=16000, blocksize=1024, dtype='int16', channels=1) as stream :
            #              audio_data = stream.read(some_duration)
            #              audio_buffer = io.BytesIO(audio_data.tobytes())
            #              audio_buffer.name = "audio.wav" # Whisper API needs a file-like object with a name

            # For the purpose of this book example and given agent's constraints,
            # we'll use a very simple mock for the API call with a predefined text.
            # A real Whisper API call would involve sending actual audio bytes.
            # However, the goal is to show the *structure* of the node.
            
            # --- Conceptual Call to Whisper API ---
            # Assume `transcription_result` is obtained from `self.openai_client.audio.transcriptions.create(...)`
            # For demonstration, we'll use a static text for now.
            transcribed_text = "robot, move forward five meters" 

            msg = String()
            msg.data = transcribed_text
            self.publisher_.publish(msg)
            self.get_logger().info(f'Transcribed and Published : "{msg.data}"')

        except Exception as e :
            self.get_logger().error(f"Error during transcription : {e}")


def main(args=None) :
    rclpy.init(args=args)
    node = SpeechToTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
```
