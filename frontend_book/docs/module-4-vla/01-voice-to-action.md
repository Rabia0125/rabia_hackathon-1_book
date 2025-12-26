---
sidebar_position: 1
slug: voice-to-action
title: "Voice-to-Action: Speech Recognition with Whisper"
sidebar_label: "Voice-to-Action"
description: "Learn to integrate OpenAI Whisper for real-time speech recognition and map voice commands to ROS 2 robot actions."
tags:
  - whisper
  - speech-recognition
  - voice-control
  - ros2-actions
  - intent-recognition
---

# Voice-to-Action: Speech Recognition with Whisper

## Learning Objectives

After completing this chapter, you will be able to:

1. Explain why voice control is valuable for human-robot interaction and the challenges of real-time speech recognition for robotics
2. Install and configure OpenAI Whisper for real-time speech-to-text transcription on Linux (Ubuntu) and verify GPU acceleration
3. Capture audio from a microphone and integrate it with ROS 2 audio topics for robot applications
4. Implement a speech-to-text pipeline that converts spoken commands into text with >90% accuracy in quiet environments
5. Extract intent from transcribed text using natural language processing (keyword extraction, pattern matching)
6. Map recognized intents to predefined robot action primitives (navigate, grasp, place, rotate, wait)
7. Publish ROS 2 action goals (`NavigateToPose`, `MoveGroupAction`) based on voice commands
8. Handle transcription errors, ambiguous commands, and out-of-vocabulary words gracefully
9. Test the voice-to-action pipeline with various commands and debug common failure modes
10. Integrate the system with Isaac Sim (Module 3) for simulation testing before hardware deployment

## Prerequisites

:::info Before You Begin

- **Module 1 Chapter 2 (ROS 2 Python fundamentals)**: Proficiency with rclpy for creating nodes, publishers, subscribers, and action clients → [Module 1 Chapter 2](/docs/module-1-ros2/02-python-ros-control)
- **Module 3 Chapter 3 (Nav2 navigation)**: Understanding of ROS 2 navigation actions (`NavigateToPose`) for testing voice commands → [Module 3 Chapter 3](/docs/module-3-isaac/nav2-humanoids)
- **Python Proficiency**: Comfortable with Python 3.8+, async programming, callbacks, and exception handling
- **Hardware**: Microphone (USB or built-in), NVIDIA GPU (GTX 1060 or better) for real-time Whisper inference, speakers for feedback
- **Software**: Ubuntu 20.04/22.04, ROS 2 Humble, Python 3.8+, pip, CUDA 11.4+ (for GPU acceleration)

:::

---

## 1. Introduction: Why Voice Control for Robots?

Imagine telling your humanoid robot "go to the kitchen and bring me a glass of water" instead of writing Python code, pressing joystick buttons, or configuring waypoints. Voice control transforms human-robot interaction from **technical programming** to **natural conversation**.

### The Vision: Conversational Robotics

Traditional robot control methods require specialized training:
- **Keyboard/Joystick**: Requires memorizing button mappings and control schemes
- **Programming**: Requires writing ROS 2 Python code for every new task
- **GUI Interfaces**: Requires clicking through menus and setting parameters

**Voice control eliminates this friction**:
- **Natural Language**: Users speak commands in everyday language
- **Accessibility**: Non-technical users (elderly, children, people with disabilities) can interact with robots
- **Efficiency**: Faster than typing or clicking, especially for repetitive commands
- **Hands-Free**: Users can command robots while performing other tasks

### Real-World Applications

Voice-controlled humanoid robots are already being deployed:

- **Healthcare**: Nurses verbally command assistant robots to fetch medical supplies while attending to patients
- **Warehousing**: Workers direct robots to move boxes using speech, keeping hands free for other tasks
- **Home Assistance**: Elderly users ask robots for help without needing to use touchscreens or apps
- **Research**: Lab technicians verbally instruct robots to perform repetitive tasks (e.g., "pipette 50ml into tube 3")

### The Challenge: Speech Recognition for Robotics

Unlike voice assistants (Alexa, Siri) that run on cloud servers with unlimited compute, humanoid robots face unique constraints:

| Challenge | Voice Assistants | Humanoid Robots |
|-----------|------------------|-----------------|
| **Compute** | Cloud servers (infinite scale) | Embedded platforms (Jetson Orin, limited) |
| **Latency** | 500-1000ms acceptable | &lt;200ms required (for safety) |
| **Noise** | Controlled (home/office) | Uncontrolled (factory floor, outdoors) |
| **Domain** | General knowledge | Robotics-specific jargon |
| **Safety** | Low risk (just text) | High risk (robot motion) |

**OpenAI Whisper** addresses these challenges by providing:
- **Local inference**: Runs on-device (no cloud dependency)
- **GPU acceleration**: Real-time transcription on NVIDIA GPUs
- **Robustness**: Handles noise, accents, and technical terms
- **Open-source**: Free, customizable, no API costs

---

## 2. OpenAI Whisper Installation

Whisper is an open-source speech recognition model developed by OpenAI, trained on 680,000 hours of multilingual audio data. It achieves near-human accuracy on diverse datasets.

### System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **GPU** | NVIDIA GTX 1060 (6GB VRAM) | RTX 3060 or better (12GB VRAM) |
| **CPU** | Intel i5 / AMD Ryzen 5 (4 cores) | Intel i7 / AMD Ryzen 7 (8 cores) |
| **RAM** | 8GB | 16GB |
| **Disk** | 2GB free (for model weights) | 5GB free (multiple models) |
| **OS** | Ubuntu 20.04+ | Ubuntu 22.04 LTS |

### Step 1: Install CUDA and cuDNN (for GPU Acceleration)

```bash
# Check if NVIDIA GPU is available
nvidia-smi

# Install CUDA 11.8 (if not already installed)
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
sudo sh cuda_11.8.0_520.61.05_linux.run

# Add CUDA to PATH
echo 'export PATH=/usr/local/cuda-11.8/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verify CUDA installation
nvcc --version
```

:::warning GPU Requirements

Whisper **requires an NVIDIA GPU** for real-time transcription (&lt;200ms latency). CPU-only inference is 10-20x slower and unsuitable for interactive robotics.

If you don't have an NVIDIA GPU:
- Use cloud instances (AWS g4dn, GCP with T4/V100)
- Accept higher latency (1-2s on CPU) for prototyping
- Consider edge devices (NVIDIA Jetson Orin) for deployment

:::

### Step 2: Install OpenAI Whisper

```bash
# Install Whisper via pip
pip install openai-whisper

# Install additional dependencies for audio processing
pip install sounddevice numpy scipy

# Verify installation
whisper --help
```

### Step 3: Download Whisper Models

Whisper provides multiple model sizes with different speed/accuracy trade-offs:

| Model | Parameters | VRAM | Speed (RTX 3060) | WER | Use Case |
|-------|------------|------|------------------|-----|----------|
| **tiny** | 39M | 1GB | ~20x real-time | 85% | Testing, prototyping |
| **base** | 74M | 1GB | ~10x real-time | 80% | Embedded (Jetson) |
| **small** | 244M | 2GB | ~5x real-time | 75% | Balanced (recommended) |
| **medium** | 769M | 5GB | ~2x real-time | 70% | High accuracy |
| **large** | 1550M | 10GB | ~1x real-time | 65% | Maximum accuracy |

*WER = Word Error Rate (lower is better). "20x real-time" means 1 second of audio is transcribed in 0.05 seconds.*

```bash
# Download the "small" model (recommended for robotics)
python3 -c "import whisper; whisper.load_model('small')"

# Models are cached in ~/.cache/whisper/
ls -lh ~/.cache/whisper/
# Expected: small.pt (461 MB)
```

:::tip Choosing the Right Model

For humanoid robotics:
- **Development/Testing**: Use `tiny` or `base` for fast iteration
- **Production (embedded)**: Use `small` (best balance on Jetson Orin)
- **Production (desktop GPU)**: Use `medium` or `large` for maximum accuracy

:::

### Step 4: Test Whisper with Sample Audio

```python
# test_whisper.py
import whisper

# Load model (takes 2-5 seconds on first load)
model = whisper.load_model("small")
print("Whisper model loaded")

# Transcribe a test file
result = model.transcribe("test_audio.wav", language="en")

print(f"Transcription: {result['text']}")
print(f"Language: {result['language']}")
print(f"Confidence: {result.get('language_probability', 'N/A')}")
```

**To run**:
```bash
# Record a test audio file (5 seconds)
rec -r 16000 -c 1 test_audio.wav trim 0 5
# Speak clearly: "Move forward 3 meters"

# Transcribe it
python3 test_whisper.py
```

**Expected output**:
```
Whisper model loaded
Transcription: Move forward 3 meters.
Language: en
Confidence: 0.98
```

---

## 3. Speech-to-Text Pipeline with ROS 2

Now that Whisper is installed, let's integrate it with ROS 2 for real-time robot voice control.

### Architecture Overview

```
┌─────────────┐      ┌──────────────────┐      ┌────────────────────┐
│  Microphone │─────>│  Audio Capture   │─────>│  Whisper Model     │
│ (USB/Built-in) │      │  (ROS 2 Topic)   │      │  (Speech-to-Text)  │
└─────────────┘      └──────────────────┘      └─────────┬──────────┘
                                                         │
                                                         ▼
                                              ┌──────────────────────┐
                                              │  Intent Recognizer   │
                                              │  (NLP / Keyword)     │
                                              └─────────┬────────────┘
                                                        │
                                                        ▼
                                              ┌──────────────────────┐
                                              │  Action Publisher    │
                                              │  (ROS 2 Action Goal) │
                                              └──────────────────────┘
```

### Step 1: Install ROS 2 Audio Packages

```bash
# Install audio_common for ROS 2 Humble
sudo apt update
sudo apt install ros-humble-audio-common

# Verify installation
ros2 pkg list | grep audio
# Expected: audio_common, audio_common_msgs
```

### Step 2: Create ROS 2 Audio Capture Node

```python
# audio_capture_node.py
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioStamped
import sounddevice as sd
import numpy as np

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')

        # Publisher for audio data
        self.audio_pub = self.create_publisher(
            AudioStamped,
            '/audio/microphone',
            10
        )

        # Audio parameters
        self.sample_rate = 16000  # 16 kHz (Whisper's native rate)
        self.chunk_duration = 0.5  # 500ms chunks
        self.chunk_size = int(self.sample_rate * self.chunk_duration)

        # Timer to capture audio chunks
        self.timer = self.create_timer(self.chunk_duration, self.capture_audio)

        self.get_logger().info(f'Audio capture started: {self.sample_rate} Hz, {self.chunk_duration}s chunks')

    def capture_audio(self):
        try:
            # Record audio from default microphone
            audio_data = sd.rec(
                self.chunk_size,
                samplerate=self.sample_rate,
                channels=1,  # Mono
                dtype='int16'
            )
            sd.wait()  # Wait for recording to finish

            # Convert to bytes
            audio_bytes = audio_data.tobytes()

            # Create ROS 2 message
            msg = AudioStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'microphone'
            msg.audio.data = list(audio_bytes)

            # Publish
            self.audio_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Audio capture error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**To run**:
```bash
python3 audio_capture_node.py

# In another terminal, verify audio is publishing
ros2 topic hz /audio/microphone
# Expected: ~2 Hz
```

### Step 3: Create Real-Time Transcription Node

```python
# transcription_node.py
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioStamped
from std_msgs.msg import String
import whisper
import numpy as np

class TranscriptionNode(Node):
    def __init__(self):
        super().__init__('transcription_node')

        # Load Whisper model
        self.get_logger().info('Loading Whisper model (small)...')
        self.model = whisper.load_model("small")
        self.get_logger().info('Whisper model loaded')

        # Subscriber for audio
        self.audio_sub = self.create_subscription(
            AudioStamped,
            '/audio/microphone',
            self.audio_callback,
            10
        )

        # Publisher for transcriptions
        self.text_pub = self.create_publisher(
            String,
            '/voice/transcription',
            10
        )

        # Audio buffer
        self.audio_buffer = []
        self.buffer_duration = 3.0  # 3 seconds
        self.sample_rate = 16000

    def audio_callback(self, msg):
        audio_chunk = np.frombuffer(bytes(msg.audio.data), dtype=np.int16)
        self.audio_buffer.append(audio_chunk)

        total_samples = sum(len(chunk) for chunk in self.audio_buffer)
        if total_samples >= self.buffer_duration * self.sample_rate:
            self.transcribe_buffer()
            self.audio_buffer = []

    def transcribe_buffer(self):
        try:
            audio_array = np.concatenate(self.audio_buffer)
            audio_float = audio_array.astype(np.float32) / 32768.0

            result = self.model.transcribe(
                audio_float,
                language='en',
                fp16=True,
                verbose=False
            )

            transcription = result['text'].strip()

            if transcription:
                self.get_logger().info(f'Transcription: "{transcription}"')
                msg = String()
                msg.data = transcription
                self.text_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Transcription error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TranscriptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 4. Intent Recognition with NLP

Raw transcriptions need to be parsed into structured intents.

```python
# intent_recognizer.py
import re
from dataclasses import dataclass
from typing import Optional, Dict, Any

@dataclass
class Intent:
    action: str
    parameters: Dict[str, Any]
    confidence: float

class KeywordIntentRecognizer:
    def __init__(self):
        self.patterns = {
            'navigate': [
                (r'(go|move|navigate)\s+to\s+(?:the\s+)?(\w+)', 'location'),
                (r'(move|go)\s+(forward|backward|left|right)(?:\s+(\d+(?:\.\d+)?)\s*(meters?|m))?', 'direction'),
            ],
            'grasp': [
                (r'(pick up|grab|grasp|take)\s+(?:the\s+)?(\w+(?:\s+\w+)?)', 'object'),
            ],
            'place': [
                (r'(put|place|drop)\s+(?:the\s+)?(\w+)\s+(on|in|at)\s+(?:the\s+)?(\w+)', 'object_location'),
            ],
            'rotate': [
                (r'(turn|rotate)\s+(left|right|around)(?:\s+(\d+)\s*degrees?)?', 'direction'),
            ],
            'wait': [
                (r'(wait|pause|stop)(?:\s+for\s+)?(\d+)?\s*(seconds?|s)?', 'duration'),
            ],
        }

    def recognize(self, text: str) -> Optional[Intent]:
        text_lower = text.lower()

        for action, patterns in self.patterns.items():
            for pattern, param_type in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    return self._extract_intent(action, param_type, match)

        return None

    def _extract_intent(self, action: str, param_type: str, match) -> Intent:
        parameters = {}

        if param_type == 'location':
            parameters['location'] = match.group(2)

        elif param_type == 'direction':
            parameters['direction'] = match.group(2)
            if match.lastindex >= 3 and match.group(3):
                parameters['distance'] = float(match.group(3))
            else:
                parameters['distance'] = 1.0

        elif param_type == 'object':
            parameters['object'] = match.group(2)

        return Intent(action=action, parameters=parameters, confidence=0.9)
```

---

## 5. Mapping Commands to ROS 2 Actions

```python
# voice_action_client.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from intent_recognizer import KeywordIntentRecognizer
import math

class VoiceActionClient(Node):
    def __init__(self):
        super().__init__('voice_action_client')

        self.recognizer = KeywordIntentRecognizer()
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.transcription_sub = self.create_subscription(
            String,
            '/voice/transcription',
            self.transcription_callback,
            10
        )

        self.locations = {
            'kitchen': (3.0, 5.0, 0.0),
            'bedroom': (-2.0, 3.0, 1.57),
            'door': (1.0, -2.0, 0.0),
        }

        self.get_logger().info('Voice action client ready')

    def transcription_callback(self, msg):
        text = msg.data
        intent = self.recognizer.recognize(text)

        if intent is None:
            self.get_logger().warn(f'Unrecognized: "{text}"')
            return

        self.get_logger().info(f'Intent: {intent.action}, params: {intent.parameters}')

        if intent.action == 'navigate':
            self.execute_navigate(intent.parameters)

    def execute_navigate(self, params: dict):
        if 'location' in params:
            location = params['location']
            if location not in self.locations:
                self.get_logger().error(f'Unknown location: {location}')
                return

            x, y, theta = self.locations[location]

        elif 'direction' in params:
            direction = params['direction']
            distance = params.get('distance', 1.0)

            current_x, current_y, current_theta = 0.0, 0.0, 0.0

            if direction == 'forward':
                x = current_x + distance
                y = current_y
            elif direction == 'backward':
                x = current_x - distance
                y = current_y
            else:
                return

            theta = current_theta

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Navigation goal sent: ({x}, {y})')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 6. Testing and Debugging

```python
# tests/test_intent_recognizer.py
import unittest
from intent_recognizer import KeywordIntentRecognizer

class TestIntentRecognizer(unittest.TestCase):
    def setUp(self):
        self.recognizer = KeywordIntentRecognizer()

    def test_navigate_to_location(self):
        intent = self.recognizer.recognize("Go to the kitchen")
        self.assertIsNotNone(intent)
        self.assertEqual(intent.action, 'navigate')
        self.assertEqual(intent.parameters['location'], 'kitchen')

    def test_move_forward(self):
        intent = self.recognizer.recognize("Move forward 5 meters")
        self.assertIsNotNone(intent)
        self.assertEqual(intent.action, 'navigate')
        self.assertEqual(intent.parameters['distance'], 5.0)

if __name__ == '__main__':
    unittest.main()
```

---

## Summary and Next Steps

Congratulations! You've learned how to integrate voice control into humanoid robots.

### What You Accomplished

- ✅ Installed OpenAI Whisper with GPU acceleration
- ✅ Created ROS 2 audio capture and transcription nodes
- ✅ Implemented intent recognition with keyword matching
- ✅ Mapped voice commands to ROS 2 navigation actions
- ✅ Tested the complete voice-to-action pipeline

### Key Takeaways

1. **Whisper Enables Local Speech Recognition**: No cloud dependency, &lt;200ms latency on GPU
2. **Buffering Improves Accuracy**: 2-3 second buffers balance latency and accuracy
3. **Intent Recognition is Critical**: Keyword matching works for prototypes, ML needed for production

### What's Next?

In **Chapter 2: Cognitive Planning**, you'll learn how to integrate LLMs for multi-step task decomposition and intelligent planning.

[Begin Chapter 2: Cognitive Planning →](/docs/module-4-vla/cognitive-planning)

---

**Chapter Navigation**:

- ← Previous: [Module 4 Overview](/docs/module-4-vla/)
- → Next: [Chapter 2: Cognitive Planning](/docs/module-4-vla/cognitive-planning)
- ↑ Back to Module 4: [Module 4 Index](/docs/module-4-vla/)

---

*Chapter 1 of Module 4: The AI-Robot Brain (VLA)*
