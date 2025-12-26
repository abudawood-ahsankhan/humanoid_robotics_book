#!/usr/bin/env python3

"""
Whisper Processing Node for VLA System

This node processes audio data using OpenAI Whisper for speech recognition.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData
from std_msgs.msg import String
from vla_msgs.msg import Transcription
import numpy as np
import io
import wave
import whisper


class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Create subscriber for audio input
        self.audio_sub = self.create_subscription(
            AudioData,
            '/vla/audio_chunk',
            self.audio_callback,
            10
        )

        # Create subscriber for audio from voice command node
        self.voice_audio_sub = self.create_subscription(
            AudioData,
            '/audio_input',
            self.voice_audio_callback,
            10
        )

        # Create publisher for transcription text
        self.text_pub = self.create_publisher(String, '/vla/command_text', 10)

        # Create publisher for detailed transcription
        self.transcription_pub = self.create_publisher(Transcription, '/vla/transcription', 10)

        # Load Whisper model (using small model for faster processing)
        try:
            self.model = whisper.load_model("small")
            self.get_logger().info('Whisper model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            self.model = None

        self.get_logger().info('Whisper Node initialized')

    def audio_callback(self, msg):
        """Process audio chunk from topic"""
        if self.model is None:
            return

        try:
            # Convert audio data to numpy array
            audio_array = np.frombuffer(msg.data, dtype=np.int16)

            # Convert to float32 in range [-1, 1]
            audio_array = audio_array.astype(np.float32) / 32768.0

            # Transcribe audio
            result = self.model.transcribe(audio_array)

            # Create and publish transcription
            text_msg = String()
            text_msg.data = result['text']
            self.text_pub.publish(text_msg)

            # Create and publish detailed transcription
            transcription_msg = Transcription()
            transcription_msg.text = result['text']
            transcription_msg.confidence = 0.9  # Whisper doesn't provide confidence, using default
            transcription_msg.timestamp = self.get_clock().now().to_msg()
            transcription_msg.language = result.get('language', 'unknown')
            self.transcription_pub.publish(transcription_msg)

            self.get_logger().info(f'Transcribed: {result["text"]}')

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def voice_audio_callback(self, msg):
        """Process audio from voice command node"""
        # For now, just pass it to the same processing function
        self.audio_callback(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()