#!/usr/bin/env python3
"""
Test script to check the actual dimensions of embeddings returned by the Gemini service.
"""

import asyncio
import sys
import os

# Add the backend directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.services.gemini_service import gemini_service
from src.config.settings import settings


async def test_embedding_dimensions():
    """Test the actual dimensions of embeddings."""
    print("Testing embedding dimensions...")
    print(f"Embedding model: {settings.embedding_model}")

    test_text = "This is a test sentence for embedding."

    try:
        embedding = await gemini_service.generate_embedding(test_text)
        print(f"Embedding length: {len(embedding)} dimensions")
        print(f"First 10 values: {embedding[:10]}")
        print(f"Sample values: {embedding[100:110] if len(embedding) > 100 else 'N/A'}")
    except Exception as e:
        print(f"Error generating embedding: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(test_embedding_dimensions())