# Chapter 13: Voice Commands → Whisper → ROS 2

## Learning Goals
- Understand the components of a voice-controlled robot system.
- Learn how to integrate speech-to-text models (e.g., OpenAI Whisper) into a robotics pipeline.
- Comprehend the process of translating natural language commands into executable ROS 2 actions.
- Explore techniques for robust command interpretation and error handling.
- Gain practical experience with building a basic voice interface for a simulated robot.

## Prerequisites
Familiarity with ROS 2 core concepts (Module 1), Python programming, and VLA systems (Chapter 12). Basic audio processing knowledge is beneficial.

## Key Concepts

### Introduction to Voice-Controlled Robotics
Voice control offers a natural and intuitive way for humans to interact with robots, moving beyond traditional programming interfaces. A voice-controlled robot system typically involves several stages: capturing audio, transcribing speech to text, interpreting the text command, and finally executing the corresponding robot action. This chapter focuses on integrating powerful speech-to-text models and linking them to ROS 2 for physical action.

### Speech-to-Text with OpenAI Whisper
**OpenAI Whisper** is a robust automatic speech recognition (ASR) model capable of transcribing audio into text with high accuracy, even in noisy environments and across multiple languages. It's an excellent choice for robotics applications due to its performance and flexibility.
-   **Architecture**: Whisper is a transformer-based encoder-decoder model trained on a massive dataset of diverse audio and text.
-   **Usage**: It can be used via Python libraries, local installations, or cloud APIs to convert spoken commands into text strings.
-   **Advantages**: Handles accents, background noise, and technical jargon relatively well, making it suitable for real-world robot environments.

### Natural Language Understanding (NLU) for Robotics
Once speech is transcribed, the next step is to understand the user's intent and extract relevant entities from the text command. This is the domain of Natural Language Understanding (NLU).
-   **Intent Recognition**: Identifying the primary goal of the command (e.g., `navigate`, `grasp`, `report_status`).
-   **Entity Extraction**: Identifying key parameters (e.g., `object_name` = "red cube", `location` = "kitchen", `action_modifier` = "slowly").
-   **Dialogue Management**: For multi-turn conversations, the system needs to maintain context and ask clarifying questions.

LLMs (from Chapter 12) are increasingly used for advanced NLU in robotics, as they can perform both intent recognition and entity extraction, and even generate executable code snippets from complex instructions.

### Translating Commands to ROS 2 Actions
After NLU, the interpreted command needs to be translated into a sequence of ROS 2 actions, services, or topic messages that the robot can understand and execute.
-   **Action Mapping**: A predefined mapping (or an LLM-generated one) connects recognized intents and entities to specific ROS 2 communication primitives.
-   **Parameter Filling**: Extracted entities fill the parameters of ROS 2 messages or action goals (e.g., `navigate_to_waypoint.action.Goal(x=1.0, y=2.5)`).
-   **Execution Orchestration**: A central node (or behavior tree) orchestrates the execution of these ROS 2 commands, potentially using feedback to adjust the plan.

### Robustness and Error Handling
Real-world voice control needs to be robust to errors:
-   **ASR Errors**: Misinterpretations by Whisper (e.g., "pick up the blue pen" vs. "pick up the glue pen").
-   **NLU Ambiguity**: Commands that are inherently unclear (e.g., "move that thing over there").
-   **Execution Failures**: Robot unable to perform the action (e.g., object out of reach, path blocked).

Strategies include:
-   **Confidence Scoring**: Using Whisper's confidence scores to prompt clarification if transcription is uncertain.
-   **Clarification Dialogues**: Asking the user follow-up questions to resolve ambiguities.
-   **Fallback Behaviors**: Implementing safe default actions or error recovery routines.
-   **Visual Confirmation**: Using robot vision to confirm understanding (e.g., "Do you mean this red cube?").

## Diagrams
-   **Diagram 1: Voice command processing pipeline**
    *   *Description*: A flow diagram illustrating the end-to-end process of a voice-controlled robot. Start with Audio Input (Microphone). Show the path through Speech-to-Text (Whisper), then Natural Language Understanding (NLU/LLM), Command Translation, and finally to Robot Actions (ROS 2 Topics/Services/Actions). Include feedback loops for clarification or visual confirmation.

## Examples

### Basic Voice Command to ROS 2 `cmd_vel` (Conceptual)
This example outlines a simple system where voice commands control a simulated robot's movement.

1.  **Audio Capture**: A microphone captures user speech.
2.  **Whisper Transcription (Python)**:

    ```python
    import whisper
    import pyaudio
    import wave

    # Load Whisper model (e.g., tiny, base, small, medium, large)
    model = whisper.load_model("base")

    # --- Simplified Audio Recording (conceptual) ---
    # This part would involve real-time audio chunking and processing
    # For demonstration, assume we have an audio file 'command.wav'
    # Or use a library like SpeechRecognition to handle live audio

    # Transcribe audio file
    # result = model.transcribe("command.wav")
    # text_command = result["text"]

    text_command = "robot move forward"
    print(f"Transcribed: {text_command}")
    ```

3.  **Command Interpretation (Python)**:
    *   A simple rule-based or LLM-driven interpreter extracts intent and parameters.

    ```python
    # simple_interpreter.py
    def interpret_command(text):
        text = text.lower()
        if "move forward" in text:
            return {"intent": "move", "direction": "forward", "speed": 0.2}
        elif "turn left" in text:
            return {"intent": "turn", "direction": "left", "angular_speed": 0.5}
        # ... more rules ...
        return {"intent": "unknown"}

    interpreted = interpret_command(text_command)
    print(f"Interpreted: {interpreted}")
    ```

4.  **ROS 2 `cmd_vel` Publisher (Python)**:
    *   A ROS 2 node publishes `Twist` messages to the `/cmd_vel` topic.

    ```python
    # ros2_cmd_vel_publisher.py
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    import time

    class CmdVelPublisher(Node):
        def __init__(self):
            super().__init__('cmd_vel_publisher')
            self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
            self.twist_msg = Twist()
            self.get_logger().info('CmdVel Publisher node started')

        def publish_command(self, linear_x=0.0, angular_z=0.0):
            self.twist_msg.linear.x = float(linear_x)
            self.twist_msg.angular.z = float(angular_z)
            self.publisher_.publish(self.twist_msg)
            self.get_logger().info(f'Publishing: linear_x={linear_x}, angular_z={angular_z}')

    # --- Integration Logic (conceptual) ---
    # In a real system, the interpreter's output would call this publisher
    # For example:
    # if interpreted["intent"] == "move":
    #     publisher_node.publish_command(linear_x=interpreted["speed"])
    # elif interpreted["intent"] == "turn":
    #     publisher_node.publish_command(angular_z=interpreted["angular_speed"])
    ```

## Hands-on Exercises

### Exercise 1: Integrating Whisper for speech recognition
1.  **Install Whisper**: Install OpenAI Whisper in a Python environment: `pip install -U openai-whisper`.
2.  **Install PyAudio**: For live audio capture, you'll need PyAudio and `wave`: `pip install pyaudio`.
3.  **Basic Transcription**: Write a Python script that records a short audio clip (e.g., 5 seconds) using PyAudio, saves it to a `.wav` file, and then uses the Whisper `base` model to transcribe the audio to text. Test with simple commands like "hello robot" or "move forward."
4.  **Real-time Transcription (Optional)**: Explore libraries like `SpeechRecognition` or `pyaudio` for real-time audio chunk processing to achieve near-instant transcription.

### Exercise 2: Mapping text commands to ROS 2 `cmd_vel`
1.  **Simulate a ROS 2 Robot**: Launch a simple wheeled robot in Isaac Sim or Gazebo that subscribes to `/cmd_vel` to move.
2.  **Create a ROS 2 Python Node**: Develop a Python ROS 2 node that:
    *   Receives text commands (e.g., from a simulated Whisper output).
    *   Interprets these commands (using simple `if/elif` statements for "move forward," "turn left," "stop").
    *   Publishes appropriate `geometry_msgs/msg/Twist` messages to `/cmd_vel` to control the robot.
3.  **Test**: Manually feed text commands to your Python node (e.g., via a separate script publishing to a topic that your interpreter node subscribes to) and observe the robot's movement in the simulator.

## Assignments

1.  **Advanced Voice Control for Humanoid Manipulation**: Design a voice command system for a humanoid robot in Isaac Sim to perform a pick-and-place task (e.g., "robot, pick up the blue cup and place it on the table"). Your design should include:
    *   A conceptual pipeline using Whisper for speech-to-text.
    *   An NLU component (e.g., an LLM or a more sophisticated rule-based system) for intent and entity extraction.
    *   Mapping these interpreted commands to a sequence of ROS 2 actions/services for navigation (Nav2) and manipulation (e.g., `moveit_msgs/action/Pick`, `moveit_msgs/action/Place`).
    *   Strategies for handling ambiguity or execution failures. Provide a detailed `mermaid` diagram and conceptual Python snippets.

2.  **Context-Aware Voice Commands**: Explain how a voice-controlled robot could become context-aware. For example, how could it interpret "pick it up" after a user has just pointed to an object? Discuss:
    *   The role of multimodal perception (vision + language).
    *   Maintaining dialogue history or state.
    *   Using environmental information (e.g., what objects are currently visible) to resolve ambiguous pronouns or references.
    *   How an LLM could facilitate this context-awareness. Provide examples of ambiguous commands and how your proposed system would resolve them.

## Summary
Chapter 13 explored the development of voice-controlled robotics systems, focusing on the pipeline from raw audio input to physical robot action. We delved into the capabilities of OpenAI Whisper for accurate speech-to-text transcription and discussed the crucial role of Natural Language Understanding (NLU) in interpreting user intent and extracting command parameters. The chapter then detailed how these interpreted commands are translated into executable ROS 2 actions, services, or topics, enabling robots to perform tasks based on spoken instructions. Practical examples and exercises guided the integration of these components, emphasizing the importance of robustness and error handling for real-world deployment, and paving the way for more natural and intuitive human-robot interaction in physical AI systems.
