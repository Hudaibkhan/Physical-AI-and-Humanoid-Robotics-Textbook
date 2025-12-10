import google.generativeai as genai
from typing import List, Dict, Any, Optional
import os
from dotenv import load_dotenv
import logging
from pydantic import BaseModel
from ..config import get_config

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class Message(BaseModel):
    role: str  # "user" or "model"
    content: str

class GeminiLLMProvider:
    """
    Custom LLM provider that wraps Google's Gemini API to work with OpenAI Agent SDK
    """
    def __init__(self):
        gemini_api_key = os.getenv("GEMINI_API_KEY")
        if not gemini_api_key:
            raise ValueError("GEMINI_API_KEY must be set in environment variables")

        genai.configure(api_key=gemini_api_key)
        self.config = get_config()
        self.model = genai.GenerativeModel(self.config.GEMINI_MODEL)

    def chat_completions_create(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = 0.7,
        max_tokens: Optional[int] = None,
        **kwargs
    ) -> Dict[str, Any]:
        """
        Create a chat completion using Gemini, formatted to match OpenAI's API

        Args:
            messages: List of message dictionaries with 'role' and 'content' keys
            temperature: Controls randomness (0.0-1.0)
            max_tokens: Maximum number of tokens to generate
            **kwargs: Additional parameters

        Returns:
            Dictionary formatted to match OpenAI's response structure
        """
        try:
            # Convert messages to Gemini format
            gemini_messages = []
            for msg in messages:
                role = msg.get('role', 'user')
                content = msg.get('content', '')

                # Map OpenAI roles to Gemini roles
                if role == 'system':
                    # Gemini doesn't have system role, so we'll add it as a user message
                    gemini_messages.append({'role': 'user', 'parts': [content]})
                elif role == 'user':
                    gemini_messages.append({'role': 'user', 'parts': [content]})
                elif role == 'assistant':
                    gemini_messages.append({'role': 'model', 'parts': [content]})

            # Generate content using Gemini
            chat = self.model.start_chat(history=gemini_messages[:-1])  # Use all but last as history
            last_message = gemini_messages[-1]['parts'][0] if gemini_messages else ""

            response = chat.send_message(
                last_message,
                generation_config=genai.types.GenerationConfig(
                    temperature=temperature,
                    max_output_tokens=max_tokens
                ) if temperature is not None or max_tokens is not None else None
            )

            # Format response to match OpenAI's structure
            openai_formatted_response = {
                "id": "chatcmpl-" + str(hash(response.text))[:8],  # Simple ID generation
                "object": "chat.completion",
                "created": int(__import__('time').time()),
                "model": self.config.GEMINI_MODEL,
                "choices": [
                    {
                        "index": 0,
                        "message": {
                            "role": "assistant",
                            "content": response.text
                        },
                        "finish_reason": "stop"
                    }
                ],
                "usage": {
                    "prompt_tokens": len(last_message.split()),
                    "completion_tokens": len(response.text.split()),
                    "total_tokens": len(last_message.split()) + len(response.text.split())
                }
            }

            logger.info(f"Gemini response generated successfully")
            return openai_formatted_response

        except Exception as e:
            logger.error(f"Error generating Gemini response: {str(e)}")
            raise

    def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """
        Generate a response to a prompt with optional context

        Args:
            prompt: The user's prompt
            context: Optional context to include in the response

        Returns:
            Generated response text
        """
        try:
            if context:
                full_prompt = f"Context: {context}\n\nQuestion: {prompt}"
            else:
                full_prompt = prompt

            response = self.model.generate_content(full_prompt)
            return response.text
        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            raise

# Global instance for reuse
gemini_provider = GeminiLLMProvider()

def get_gemini_provider() -> GeminiLLMProvider:
    """
    Get the Gemini LLM provider instance
    """
    return gemini_provider