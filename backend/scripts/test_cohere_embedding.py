#!/usr/bin/env python3
"""
Test script to check Cohere embedding functionality.
"""

import asyncio
import sys
import os

# Add the backend directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.services.gemini_service import gemini_service
from src.config.settings import settings


async def test_cohere_embedding():
    """Test Cohere embedding functionality."""
    print("Testing Cohere embedding functionality...")
    print(f"Embedding model: {settings.embedding_model}")

    test_text = "This is a test sentence for Cohere embedding."

    try:
        embedding = await gemini_service.generate_embedding(test_text)
        print(f"Embedding length: {len(embedding)} dimensions")
        print(f"First 10 values: {embedding[:10]}")
        print(f"Sample values: {embedding[100:110] if len(embedding) > 100 else 'N/A'}")

        # Check if it's the fallback vector (all zeros)
        if all(v == 0.0 for v in embedding):
            print("WARNING: Returned vector is all zeros - may be fallback due to error")
        else:
            print("SUCCESS: Cohere embedding generated successfully")
    except Exception as e:
        print(f"Error generating embedding: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(test_cohere_embedding())