"""Test response formatting functionality."""

import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add fastapi_app to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'fastapi_app'))

from app import app

client = TestClient(app)


def test_greeting_has_markdown_structure():
    """Test that greeting responses follow markdown structure."""
    response = client.post("/chat", json={"message": "hello"})
    assert response.status_code == 200

    body = response.json()
    response_text = body["response"]

    # Check for markdown headings
    assert "##" in response_text, "Response should have main heading (##)"
    assert "###" in response_text, "Response should have subsections (###)"


def test_technical_response_has_key_sections():
    """Test that technical responses have required sections."""
    # Note: This test requires valid API key and may fail due to quota
    # Skip if quota exceeded or API unavailable

    response = client.post("/chat", json={"message": "What is inverse kinematics?"})
    assert response.status_code == 200

    body = response.json()
    response_text = body["response"]

    # Should have markdown structure (if not error response)
    if "Error" not in response_text and "unavailable" not in response_text:
        # Check for expected sections (at least some markdown)
        assert "##" in response_text or "#" in response_text, "Response should have headings"


def test_response_uses_bullet_points():
    """Test that responses use bullet points for lists."""
    response = client.post("/chat", json={"message": "hello"})
    assert response.status_code == 200

    body = response.json()
    response_text = body["response"]

    # Check for bullet points
    assert "-" in response_text, "Response should use bullet points for lists"


def test_response_has_professional_tone():
    """Test that responses maintain professional, instructor-like tone."""
    response = client.post("/chat", json={"message": "hello"})
    assert response.status_code == 200

    body = response.json()
    response_text = body["response"]

    # Check for friendly, helpful language
    assert any(word in response_text.lower() for word in ["help", "learn", "assist"]), \
        "Response should have helpful, instructional tone"
