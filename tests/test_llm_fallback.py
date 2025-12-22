"""Test LLM fallback functionality."""

import pytest
from unittest.mock import patch, AsyncMock
import sys
import os

# Add fastapi_app to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'fastapi_app'))

from llm_router import LLMRouter, QuotaExceededError


class MockAgentResult:
    """Mock agent result for testing."""
    def __init__(self, output):
        self.final_output = output
        self.output = output


@pytest.mark.asyncio
async def test_llm_router_quota_detection():
    """Test that LLMRouter detects quota errors."""
    # Test various quota error messages
    quota_errors = [
        "Error code: 429",
        "RESOURCE_EXHAUSTED",
        "quota exceeded",
        "rate limit",
        "too many requests"
    ]

    for error_msg in quota_errors:
        error = Exception(error_msg)
        error_str = str(error).lower()

        # Verify error detection logic
        quota_keywords = ['quota', '429', 'resource_exhausted', 'rate limit', 'too many requests']
        is_quota_error = any(keyword in error_str for keyword in quota_keywords)

        assert is_quota_error, f"Should detect '{error_msg}' as quota error"


@pytest.mark.asyncio
async def test_llm_router_fallback_on_quota_error():
    """Test that LLMRouter falls back to Groq when Gemini quota exceeded."""
    # Create mock configs
    mock_gemini_config = {"model": "gemini"}
    mock_groq_config = {"model": "groq"}

    router = LLMRouter(mock_gemini_config, mock_groq_config)

    # Mock Runner.run to simulate Gemini failure, Groq success
    with patch('llm_router.Runner.run', new_callable=AsyncMock) as mock_run:
        # First call (Gemini) raises quota error, second call (Groq) succeeds
        mock_run.side_effect = [
            Exception("RESOURCE_EXHAUSTED"),
            MockAgentResult(output="Groq fallback response")
        ]

        mock_agent = object()
        result = await router.run_with_fallback(mock_agent, "test message")

        # Verify fallback was triggered
        assert mock_run.call_count == 2, "Should call Gemini, then Groq"
        assert router.fallback_count == 1, "Should increment fallback counter"
        assert result.output == "Groq fallback response"


@pytest.mark.asyncio
async def test_llm_router_no_fallback_on_other_errors():
    """Test that LLMRouter doesn't fallback for non-quota errors."""
    mock_gemini_config = {"model": "gemini"}
    mock_groq_config = {"model": "groq"}

    router = LLMRouter(mock_gemini_config, mock_groq_config)

    with patch('llm_router.Runner.run', new_callable=AsyncMock) as mock_run:
        # Raise non-quota error
        mock_run.side_effect = Exception("Network connection error")

        mock_agent = object()

        with pytest.raises(Exception, match="Network connection error"):
            await router.run_with_fallback(mock_agent, "test message")

        # Should only try once (no fallback for non-quota errors)
        assert mock_run.call_count == 1
        assert router.fallback_count == 0


@pytest.mark.asyncio
async def test_llm_router_both_fail():
    """Test that LLMRouter raises exception when both LLMs fail."""
    mock_gemini_config = {"model": "gemini"}
    mock_groq_config = {"model": "groq"}

    router = LLMRouter(mock_gemini_config, mock_groq_config)

    with patch('llm_router.Runner.run', new_callable=AsyncMock) as mock_run:
        # Both calls fail
        mock_run.side_effect = [
            Exception("RESOURCE_EXHAUSTED"),
            Exception("Groq also failed")
        ]

        mock_agent = object()

        with pytest.raises(Exception, match="Both LLMs unavailable"):
            await router.run_with_fallback(mock_agent, "test message")

        assert mock_run.call_count == 2
        assert router.fallback_count == 1
