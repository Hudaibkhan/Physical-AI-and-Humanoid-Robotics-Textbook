"""Greeting detection module for agent behavior enhancement.

This module provides functionality to detect greeting messages before RAG execution,
allowing the agent to respond with friendly welcome messages without triggering
unnecessary vector store queries.
"""

from typing import Set


class GreetingDetector:
    """Detects greeting messages to bypass RAG retrieval."""

    # Immutable set of recognized greeting patterns (lowercase)
    GREETING_PATTERNS: Set[str] = {
        "hi",
        "hello",
        "hey",
        "salam",
        "assalam o alaikum"
    }

    @classmethod
    def is_greeting(cls, message: str) -> bool:
        """
        Check if message matches a greeting pattern.

        Performs case-insensitive exact string matching against predefined patterns.
        No partial matching (e.g., "hello world" does NOT match "hello").

        Args:
            message: User message to check

        Returns:
            True if message is a greeting, False otherwise

        Examples:
            >>> GreetingDetector.is_greeting("hello")
            True
            >>> GreetingDetector.is_greeting("SALAM")
            True
            >>> GreetingDetector.is_greeting("hello world")
            False
        """
        if not message:
            return False

        cleaned = message.lower().strip()
        return cleaned in cls.GREETING_PATTERNS

    @classmethod
    def generate_greeting_response(cls) -> str:
        """
        Generate friendly greeting response with structured markdown.

        Returns welcome message that:
        - Sets context about agent capabilities
        - Invites user to ask questions
        - Uses consistent markdown formatting
        - Does NOT reference textbook or RAG system

        Returns:
            Formatted markdown greeting message
        """
        return """## Welcome! 👋

Hello! I'm your Physical AI and Humanoid Robotics textbook assistant. I'm here to help you learn about robotics, ROS 2, and humanoid systems.

### How Can I Help?
- Explain robotics concepts from the textbook
- Answer questions about ROS 2 and navigation
- Clarify topics related to physical AI and humanoid robotics

### Why I'm Here
- Provide instant access to course content
- Support your learning journey with accurate information
- Make complex robotics concepts more accessible

Feel free to ask me anything about the course materials!"""
