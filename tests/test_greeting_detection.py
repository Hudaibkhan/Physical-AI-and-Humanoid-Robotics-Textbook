"""Test greeting detection functionality."""

import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add fastapi_app to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'fastapi_app'))

from app import app

client = TestClient(app)


def test_greeting_detection_hello():
    """Test that 'hello' is detected as greeting and bypasses RAG."""
    response = client.post("/chat", json={"message": "hello"})
    assert response.status_code == 200

    body = response.json()
    assert "Welcome" in body["response"]
    assert len(body["source_chunks"]) == 0, "Greeting should not trigger RAG retrieval"


def test_greeting_detection_hi():
    """Test that 'hi' is detected as greeting."""
    response = client.post("/chat", json={"message": "hi"})
    assert response.status_code == 200

    body = response.json()
    assert "Welcome" in body["response"]
    assert len(body["source_chunks"]) == 0


def test_greeting_detection_salam():
    """Test that 'salam' is detected as greeting."""
    response = client.post("/chat", json={"message": "salam"})
    assert response.status_code == 200

    body = response.json()
    assert "Welcome" in body["response"]
    assert len(body["source_chunks"]) == 0


def test_greeting_case_insensitive():
    """Test that greeting detection is case-insensitive."""
    response = client.post("/chat", json={"message": "HELLO"})
    assert response.status_code == 200

    body = response.json()
    assert "Welcome" in body["response"]
    assert len(body["source_chunks"]) == 0


def test_non_greeting():
    """Test that technical questions are not treated as greetings."""
    # Note: This test may fail if Gemini quota exceeded
    # It's mainly to verify greeting detection doesn't trigger for non-greetings
    response = client.post("/chat", json={"message": "What is robotics?"})
    assert response.status_code == 200

    body = response.json()
    # Should not return the greeting message
    assert "Welcome" not in body["response"] or "robotics" in body["response"].lower()
