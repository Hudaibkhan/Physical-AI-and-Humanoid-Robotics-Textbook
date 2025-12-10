#!/usr/bin/env python3
"""
Simple connection test script to verify Qdrant and Gemini connectivity
without consuming embedding quotas.
"""

import asyncio
import sys
import os

# Add the backend directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.services.qdrant_service import qdrant_service
from src.services.gemini_service import GeminiService
from src.config.settings import settings


async def test_qdrant_connection():
    """Test Qdrant connection without uploading data."""
    print("Testing Qdrant connection...")
    try:
        # Ensure collection exists (this will create it if it doesn't exist)
        qdrant_service._ensure_collection_exists()
        print("OK Qdrant connection successful!")
        print(f"   - URL: {settings.qdrant_url}")
        print(f"   - Collection: {settings.collection_name}")
        return True
    except Exception as e:
        print(f"ERROR Qdrant connection failed: {e}")
        return False


async def test_gemini_connection():
    """Test Gemini connection with a simple text generation (uses minimal quota)."""
    print("\nTesting Gemini connection...")
    try:
        gemini_service = GeminiService()
        # Test with a simple, short text generation (minimal quota usage)
        test_text = "Hello"
        response = await gemini_service.generate_response(test_text)
        if response and len(response) > 0:
            print("OK Gemini connection successful!")
            print(f"   - Response preview: {response[:50]}...")
            return True
        else:
            print("ERROR Gemini connection failed: No response")
            return False
    except Exception as e:
        print(f"ERROR Gemini connection failed: {e}")
        return False


async def main():
    print("RAG Chatbot Connection Test")
    print("=" * 50)

    qdrant_ok = await test_qdrant_connection()
    gemini_ok = await test_gemini_connection()

    print("\n" + "=" * 50)
    if qdrant_ok and gemini_ok:
        print("SUCCESS All connections successful!")
        print("\nYou can now proceed with uploading your book content.")
        print("Note: The upload process may take multiple sessions due to API quotas.")
        print("The system is designed to handle quota limits by processing in small batches.")
    else:
        print("ERROR Some connections failed. Please check your configuration.")

    print("\nServices running:")
    print(f"   - Backend API: http://localhost:8000")
    print(f"   - Frontend widget: http://localhost:3001")
    print(f"   - Book site: http://localhost:3002")


if __name__ == "__main__":
    asyncio.run(main())