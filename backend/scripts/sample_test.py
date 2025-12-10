#!/usr/bin/env python3
"""
Sample upload script to test the RAG system with a small amount of content.
This uploads just a small sample to verify the system works without consuming much quota.
"""

import asyncio
import sys
import os
from typing import List, Dict

# Add the backend directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.services.qdrant_service import qdrant_service
from src.services.gemini_service import gemini_service
from src.config.settings import settings


async def upload_sample_content():
    """Upload a small sample of content to test the system."""
    print("Uploading sample content to test the RAG system...")

    # Sample content - just a small text to test the system
    sample_chunks = [
        {
            "id": "sample-1",
            "content": "This is a sample text about Physical AI and Humanoid Robotics. This content is used to test the RAG system functionality.",
            "book_id": "sample-book",
            "chunk_index": 0,
            "metadata": {"title": "Sample Content", "page": 1}
        },
        {
            "id": "sample-2",
            "content": "ROS2 is a framework for robotics software development. It provides libraries and tools to help software developers create robot applications.",
            "book_id": "sample-book",
            "chunk_index": 1,
            "metadata": {"title": "ROS2 Introduction", "page": 2}
        },
        {
            "id": "sample-3",
            "content": "Digital twin technology allows creating virtual replicas of physical systems. This is particularly useful in robotics for simulation and testing.",
            "book_id": "sample-book",
            "chunk_index": 2,
            "metadata": {"title": "Digital Twin", "page": 3}
        }
    ]

    print(f"   - Preparing {len(sample_chunks)} sample chunks...")

    # Generate embeddings for the sample content
    print("   - Generating embeddings...")
    for chunk in sample_chunks:
        try:
            embedding = await gemini_service.generate_embedding(chunk["content"])
            if len(embedding) > 0:
                chunk["embedding"] = embedding
                print(f"     OK Generated embedding for chunk {chunk['id']}")
            else:
                print(f"     ERROR Failed to generate embedding for chunk {chunk['id']}")
                return False
        except Exception as e:
            print(f"     ERROR Error generating embedding for chunk {chunk['id']}: {e}")
            return False

    # Store the chunks in Qdrant
    print("   - Storing chunks in Qdrant...")
    try:
        success = await qdrant_service.store_chunks(sample_chunks)
        if success:
            print("   OK Sample content uploaded successfully!")
            return True
        else:
            print("   ERROR Failed to store chunks in Qdrant")
            return False
    except Exception as e:
        print(f"   ERROR Error storing chunks in Qdrant: {e}")
        return False


async def test_search():
    """Test the search functionality with a sample query."""
    print("\nTesting search functionality...")

    try:
        # Generate embedding for a test query
        query = "What is ROS2?"
        print(f"   - Query: '{query}'")

        query_embedding = await gemini_service.generate_embedding(query)
        if not query_embedding:
            print("   ERROR Failed to generate embedding for query")
            return False

        print("   - Generated query embedding")

        # Search for similar content
        results = await qdrant_service.search_similar_chunks(query_embedding, top_k=2)
        print(f"   - Found {len(results)} similar chunks")

        for i, result in enumerate(results):
            print(f"     Result {i+1}: {result['content'][:100]}...")

        return True
    except Exception as e:
        print(f"   ERROR Error during search test: {e}")
        return False


async def main():
    print("RAG System Sample Test")
    print("=" * 50)

    # Ensure the collection exists
    print("Ensuring Qdrant collection exists...")
    try:
        qdrant_service._ensure_collection_exists()
        print("   OK Collection ready")
    except Exception as e:
        print(f"   ERROR Error ensuring collection exists: {e}")
        return

    # Upload sample content
    upload_ok = await upload_sample_content()

    if upload_ok:
        # Test search functionality
        search_ok = await test_search()

        print("\n" + "=" * 50)
        if upload_ok and search_ok:
            print("Sample test completed successfully!")
            print("\nSUCCESS The RAG system is properly configured and working.")
            print("SUCCESS You can now proceed with uploading your full book content.")
            print("\nNote: For the full book upload, run:")
            print("   python scripts/upload_chunks.py")
            print("   (This may take multiple sessions due to API quotas)")
        else:
            print("ERROR Sample test failed. Please check your configuration.")
    else:
        print("ERROR Upload test failed. Please check your configuration.")


if __name__ == "__main__":
    asyncio.run(main())