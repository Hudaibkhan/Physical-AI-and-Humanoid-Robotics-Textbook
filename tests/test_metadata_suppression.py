"""Test RAG metadata suppression functionality."""

import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add fastapi_app to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'fastapi_app'))

from app import app
from formatters import ResponseFormatter

client = TestClient(app)


def test_response_formatter_strip_metadata():
    """Test that ResponseFormatter strips metadata patterns."""
    # Test various metadata patterns
    test_cases = [
        ("Answer here. Source: chunk_123", "Answer here."),
        ("Based on chunk abc, the answer is...", "the answer is..."),
        ("According to document xyz, the fact is...", "the fact is..."),
        ("Answer. Chunk ID: 456", "Answer."),
        ("Simple answer", "Simple answer"),
    ]

    for input_text, expected_output in test_cases:
        result = ResponseFormatter.strip_metadata(input_text)
        assert expected_output in result, f"Failed to strip metadata from: {input_text}"


def test_response_formatter_validate_no_metadata():
    """Test that ResponseFormatter validates metadata absence."""
    # Clean responses should pass
    assert ResponseFormatter.validate_no_metadata("This is a clean response")
    assert ResponseFormatter.validate_no_metadata("## Topic\n\n### Key Points\n- Point 1")

    # Responses with metadata should fail
    assert not ResponseFormatter.validate_no_metadata("Answer with chunk_123")
    assert not ResponseFormatter.validate_no_metadata("Source: chunk_abc in this text")
    assert not ResponseFormatter.validate_no_metadata("Based on chunk X, the answer is...")


def test_agent_response_no_metadata():
    """Test that agent responses don't contain metadata (integration test)."""
    # Note: This test requires valid API key and may fail due to quota
    # It's meant to verify end-to-end metadata suppression

    response = client.post("/chat", json={"message": "What is SLAM?"})
    assert response.status_code == 200

    body = response.json()
    response_text = body["response"].lower()

    # Check for forbidden metadata patterns
    forbidden_patterns = ["chunk_", "source:", "based on chunk", "chunk id", "similarity score"]

    for pattern in forbidden_patterns:
        assert pattern not in response_text, f"Response contains forbidden metadata: {pattern}"


def test_citations_field_removed():
    """Test that citations field is removed from response."""
    response = client.post("/chat", json={"message": "hello"})
    assert response.status_code == 200

    body = response.json()
    # Citations field should not be present
    assert "citations" not in body, "Citations field should be removed from AgentResponse"
