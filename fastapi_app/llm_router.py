"""LLM routing and fallback logic for agent behavior enhancement.

This module provides silent fallback from Gemini to Groq when quota errors occur,
ensuring service continuity without exposing infrastructure details to users.
"""

import logging
from typing import Optional, Any
from agents import Runner

logger = logging.getLogger(__name__)


class QuotaExceededError(Exception):
    """Raised when LLM API quota is exceeded."""
    pass


class LLMRouter:
    """Manages LLM selection and fallback between Gemini and Groq."""

    def __init__(self, gemini_config, groq_config):
        """
        Initialize router with primary and fallback configurations.

        Args:
            gemini_config: RunConfig for Gemini (primary LLM)
            groq_config: RunConfig for Groq (fallback LLM)
        """
        self.gemini_config = gemini_config
        self.groq_config = groq_config
        self.fallback_count = 0  # Track fallback occurrences for monitoring

    async def run_with_fallback(self, agent, input_message: str) -> Any:
        """
        Run agent with primary LLM, fallback to secondary on quota error.

        Detects quota errors (HTTP 429, RESOURCE_EXHAUSTED) and automatically
        retries with Groq without user notification. Logs fallback events for
        monitoring.

        Args:
            agent: OpenAI agent instance
            input_message: User message to process

        Returns:
            Agent execution result from either Gemini or Groq

        Raises:
            Exception: If both LLMs fail (after fallback attempt)
        """
        try:
            # Try Gemini (primary)
            result = await Runner.run(
                agent,
                input=input_message,
                run_config=self.gemini_config
            )
            return result

        except Exception as e:
            # Convert error to string and check for quota indicators
            error_str = str(e)
            error_msg = error_str.lower()

            # Also check if it's an OpenAI API error with status code
            status_code = None
            if hasattr(e, 'status_code'):
                status_code = e.status_code
            elif hasattr(e, 'code'):
                status_code = e.code

            # Check if quota/rate limit error
            quota_keywords = ['quota', '429', 'resource_exhausted', 'rate limit', 'too many requests']
            is_quota_error = any(keyword in error_msg for keyword in quota_keywords) or status_code == 429

            if is_quota_error:
                logger.warning(
                    f"Gemini quota exceeded, falling back to Groq: {e}"
                )
                self.fallback_count += 1

                # Silent fallback to Groq
                try:
                    result = await Runner.run(
                        agent,
                        input=input_message,
                        run_config=self.groq_config
                    )
                    logger.info("Groq fallback successful")
                    return result

                except Exception as groq_error:
                    logger.error(f"Groq fallback also failed: {groq_error}")
                    raise Exception("Both LLMs unavailable") from groq_error
            else:
                # Not a quota error, re-raise original exception
                raise

    def get_fallback_count(self) -> int:
        """
        Get number of times fallback was triggered.

        Returns:
            Count of Gemini → Groq fallback occurrences
        """
        return self.fallback_count
