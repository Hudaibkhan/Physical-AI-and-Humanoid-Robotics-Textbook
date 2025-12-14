"""
Simple script to test the API endpoints
"""
import requests
import json

def test_api():
    """Test the API endpoints"""
    base_url = "http://localhost:8000"

    # Test the root endpoint
    try:
        response = requests.get(f"{base_url}/")
        print(f"Root endpoint: {response.status_code} - {response.json()}")
    except Exception as e:
        print(f"Could not reach API: {e}")
        print("Make sure the FastAPI server is running with: uvicorn app:app --reload")
        return

    # Test the chat endpoint with a simple query
    chat_payload = {
        "message": "What is artificial intelligence?",
        "selected_text": None,
        "session_id": "test_session_123"
    }

    try:
        response = requests.post(f"{base_url}/chat", json=chat_payload)
        print(f"Chat endpoint (normal RAG): {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"Response: {result['response'][:100]}...")
            print(f"Source chunks: {len(result['source_chunks'])}")
            print(f"Session ID: {result['session_id']}")
        else:
            print(f"Error: {response.text}")
    except Exception as e:
        print(f"Error calling chat endpoint (normal RAG): {e}")

    # Test the chat endpoint with selected text
    chat_payload_selected = {
        "message": "What is artificial intelligence?",
        "selected_text": "Artificial intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence. This can include learning from experience, understanding natural language, solving problems, and recognizing patterns.",
        "session_id": "test_session_456"
    }

    try:
        response = requests.post(f"{base_url}/chat", json=chat_payload_selected)
        print(f"Chat endpoint (selected text): {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"Response: {result['response'][:100]}...")
            print(f"Source chunks: {len(result['source_chunks'])}")
            print(f"Session ID: {result['session_id']}")
        else:
            print(f"Error: {response.text}")
    except Exception as e:
        print(f"Error calling chat endpoint (selected text): {e}")

if __name__ == "__main__":
    test_api()