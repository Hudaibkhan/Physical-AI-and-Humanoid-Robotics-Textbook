#!/usr/bin/env python3
"""
Test script to verify that the system can connect to Qdrant and Gemini with the provided credentials
"""
import asyncio
import sys
import os

# Add the project root to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from src.services.qdrant_service import qdrant_service
from src.services.embedding_service import embedding_service
from src.config.settings import settings

async def test_connections():
    print("Testing connections with your credentials...")
    print(f"Qdrant URL: {settings.qdrant_url}")
    print(f"Gemini Model: {settings.gemini_model}")
    print()

    # Test Qdrant connection by trying to access the client directly
    print("Testing Qdrant connection...")
    try:
        # Access the client to test connection - this will trigger the collection creation if needed
        # which happens in the __init__ method of QdrantService
        collections = qdrant_service.client.get_collections()
        print(f"[OK] Qdrant connection successful! Found {len(collections.collections)} collections")

        # Check if our collection exists
        collection_exists = any(col.name == qdrant_service.collection_name for col in collections.collections)
        if collection_exists:
            print(f"[OK] Collection '{qdrant_service.collection_name}' exists")
            # Get collection info
            info = qdrant_service.client.get_collection(qdrant_service.collection_name)
            print(f"  - Points in collection: {info.points_count}")
        else:
            print(f"[INFO] Collection '{qdrant_service.collection_name}' does not exist yet (will be created when uploading)")
    except Exception as e:
        print(f"[ERROR] Qdrant connection failed: {e}")
        return False

    print()

    # Test Gemini connection
    print("Testing Gemini connection...")
    try:
        # Test embedding generation
        test_text = "This is a test sentence for embedding."
        embedding = await embedding_service.create_embedding(test_text)

        if len(embedding) > 0 and isinstance(embedding[0], (int, float)):
            print(f"[OK] Gemini connection successful! Generated embedding with {len(embedding)} dimensions")
        else:
            print("[ERROR] Gemini connection failed: Invalid embedding returned")
            return False
    except Exception as e:
        print(f"[ERROR] Gemini connection failed: {e}")
        return False

    print()
    print("[OK] All connections are working properly!")
    print("You can now run the upload script to add your book content to Qdrant.")
    return True

if __name__ == "__main__":
    success = asyncio.run(test_connections())
    if success:
        print("\nNext steps:")
        print("1. Run the upload script: python backend/scripts/upload_chunks.py")
        print("2. Start the backend: uvicorn backend/src/main:app --host 0.0.0.0 --port 8000")
        print("3. Start the frontend: npm start in the frontend directory")
        print("4. Start the book: npm start in the root directory")
    else:
        print("\nPlease check your credentials and network connection.")
        sys.exit(1)