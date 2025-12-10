from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from typing import List, Optional, Dict, Any
import logging
from pydantic import BaseModel
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class QdrantClientSingleton:
    """
    Singleton class to manage Qdrant client connection
    """
    _instance = None
    _client = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(QdrantClientSingleton, cls).__new__(cls)
        return cls._instance

    def get_client(self) -> QdrantClient:
        if self._client is None:
            qdrant_url = os.getenv("QDRANT_URL")
            qdrant_api_key = os.getenv("QDRANT_API_KEY")

            if not qdrant_url or not qdrant_api_key:
                raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables")

            self._client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                # timeout=10  # 10 seconds timeout
            )

        return self._client

    def create_collection_if_not_exists(self, collection_name: str = "book_chunks"):
        """
        Create the book_chunks collection with proper schema if it doesn't exist
        """
        client = self.get_client()

        # Check if collection exists
        try:
            collections = client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if collection_name not in collection_names:
                # Create collection with 1024-dimensional vectors for Cohere embeddings
                client.create_collection(
                    collection_name=collection_name,
                    vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
                )
                logger.info(f"Created collection '{collection_name}' with 1024-dimensional vectors")
            else:
                logger.info(f"Collection '{collection_name}' already exists")
        except Exception as e:
            logger.error(f"Error creating collection '{collection_name}': {str(e)}")
            raise

def get_qdrant_client() -> QdrantClient:
    """
    Get the Qdrant client instance
    """
    singleton = QdrantClientSingleton()
    return singleton.get_client()

def init_qdrant_collection(collection_name: str = "book_chunks"):
    """
    Initialize the Qdrant collection if it doesn't exist
    """
    singleton = QdrantClientSingleton()
    singleton.create_collection_if_not_exists(collection_name)