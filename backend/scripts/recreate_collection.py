#!/usr/bin/env python3
"""
Script to recreate the Qdrant collection with correct dimensions.
"""

import sys
import os
import asyncio

# Add the backend directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from qdrant_client import QdrantClient
from src.config.settings import settings


async def recreate_collection():
    """Recreate the Qdrant collection with correct vector dimensions."""
    print("Recreating Qdrant collection with correct dimensions...")

    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        prefer_grpc=False
    )

    collection_name = settings.collection_name

    try:
        # Try to delete the existing collection
        print(f"Deleting existing collection: {collection_name}")
        client.delete_collection(collection_name)
        print("Collection deleted successfully")
    except Exception as e:
        print(f"Collection may not have existed yet, continuing: {e}")

    # Determine the correct vector size based on the embedding model
    if settings.embedding_model.lower() == "cohere":
        vector_size = 1024  # Cohere embeddings are typically 1024-dimensional
    elif "embedding-001" in settings.embedding_model.lower():
        vector_size = 3072  # Gemini embedding-001 returns 3072-dimensional vectors
    else:
        # Default to 3072 for other models
        vector_size = 3072

    print(f"Creating collection '{collection_name}' with {vector_size}-dimensional vectors...")

    from qdrant_client.http import models
    client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(
            size=vector_size,
            distance=models.Distance.COSINE
        )
    )

    print("Collection created successfully!")
    print(f"Vector size: {vector_size} dimensions")
    print(f"Embedding model: {settings.embedding_model}")


if __name__ == "__main__":
    asyncio.run(recreate_collection())