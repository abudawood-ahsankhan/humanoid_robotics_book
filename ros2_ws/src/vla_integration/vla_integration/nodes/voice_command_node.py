#!/usr/bin/env python3

"""
Voice Command Node for VLA System

This node captures audio from a microphone and publishes it for processing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData
from std_msgs.msg import String
import pyaudio
import numpy as np
import threading
import queue


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Declare parameters
        self.declare_parameter('audio_device', 0)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('sensitivity', 0.5)

        # Get parameters
        self.audio_device = self.get_parameter('audio_device').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.sensitivity = self.get_parameter('sensitivity').value

        # Create publisher for audio data
        self.audio_pub = self.create_publisher(AudioData, '/audio_input', 10)

        # Create publisher for command text (when implemented)
        self.command_pub = self.create_publisher(String, '/vla/command_text', 10)

        # Initialize audio stream
        self.audio = pyaudio.PyAudio()
        self.audio_queue = queue.Queue()

        # Start audio capture thread
        self.capture_thread = threading.Thread(target=self.capture_audio, daemon=True)
        self.capture_thread.start()

        # Timer to process audio
        self.timer = self.create_timer(0.1, self.process_audio)

        self.get_logger().info('Voice Command Node initialized')

    def capture_audio(self):
        """Capture audio from microphone"""
        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size,
            input_device_index=self.audio_device
        )

        try:
            while rclpy.ok():
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                self.audio_queue.put(data)
        except Exception as e:
            self.get_logger().error(f'Audio capture error: {e}')
        finally:
            stream.stop_stream()
            stream.close()

    def process_audio(self):
        """Process audio data from queue"""
        try:
            while not self.audio_queue.empty():
                audio_data = self.audio_queue.get_nowait()

                # Convert to AudioData message
                audio_msg = AudioData()
                audio_msg.data = audio_data

                # Publish audio data
                self.audio_pub.publish(audio_msg)

        except queue.Empty:
            pass
        except Exception as e:
            self.get_logger().error(f'Audio processing error: {e}')

    def destroy_node(self):
        """Clean up resources"""
        self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()