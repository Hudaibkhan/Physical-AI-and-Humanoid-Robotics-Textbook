from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
from src.config import get_config
import logging

logger = logging.getLogger(__name__)


class QdrantService:
    def __init__(self):
        config = get_config()
        self.client = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY,
            prefer_grpc=False  # Using HTTP for compatibility
        )
        self.collection_name = config.QDRANT_COLLECTION_NAME
        # Set vector size based on the embedding model being used
        if config.EMBEDDING_MODEL.lower() == "cohere":
            self.vector_size = 1024  # Cohere embeddings are typically 1024-dimensional
        elif "embedding-001" in config.EMBEDDING_MODEL.lower():
            self.vector_size = 3072  # Gemini embedding-001 returns 3072-dimensional vectors
        else:
            # Default to 3072 for other models like gemini-2.5-flash embeddings
            self.vector_size = 3072

    def _ensure_collection_exists(self):
        """Ensure the collection exists with proper configuration."""
        try:
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=self.vector_size,
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")
            else:
                logger.info(f"Collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error ensuring collection exists: {e}")
            raise

    async def store_chunks(self, chunks: List[Dict]) -> bool:
        """Store text chunks with their embeddings in Qdrant."""
        try:
            points = []
            for chunk in chunks:
                # Convert string ID to integer hash to comply with Qdrant requirements
                # Use a consistent hash to maintain ID consistency
                import hashlib
                string_id = chunk["id"]
                # Create a hash of the string ID and convert to a positive integer
                id_hash = int(hashlib.md5(string_id.encode()).hexdigest(), 16) % (10**9)

                point = models.PointStruct(
                    id=id_hash,  # Use integer ID
                    vector=chunk["embedding"],
                    payload={
                        "original_id": string_id,  # Store original ID in payload
                        "book_id": chunk["book_id"],
                        "content": chunk["content"],
                        "chunk_index": chunk["chunk_index"],
                        "metadata": chunk.get("metadata", {})
                    }
                )
                points.append(point)

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Stored {len(points)} chunks in Qdrant")
            return True
        except Exception as e:
            logger.error(f"Error storing chunks in Qdrant: {e}")
            return False

    async def search_similar_chunks(self, query_embedding: List[float], top_k: int = 5) -> List[Dict]:
        """Search for similar chunks based on the query embedding."""
        try:
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            results = []
            for hit in search_results:
                result = {
                    "id": hit.payload.get("original_id", str(hit.id)),  # Use original ID from payload
                    "content": hit.payload.get("content", ""),
                    "similarity_score": hit.score,
                    "book_id": hit.payload.get("book_id"),
                    "chunk_index": hit.payload.get("chunk_index"),
                    "metadata": hit.payload.get("metadata", {})
                }
                results.append(result)

            logger.info(f"Found {len(results)} similar chunks")
            return results
        except Exception as e:
            logger.error(f"Error searching similar chunks: {e}")
            return []

    async def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict]:
        """Retrieve a specific chunk by its ID."""
        try:
            # Convert string ID to integer hash (same as during storage)
            import hashlib
            id_hash = int(hashlib.md5(chunk_id.encode()).hexdigest(), 16) % (10**9)

            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[id_hash]
            )

            if records:
                record = records[0]
                return {
                    "id": record.payload.get("original_id", str(record.id)),  # Use original ID from payload
                    "content": record.payload.get("content", ""),
                    "book_id": record.payload.get("book_id"),
                    "chunk_index": record.payload.get("chunk_index"),
                    "metadata": record.payload.get("metadata", {})
                }
            return None
        except Exception as e:
            logger.error(f"Error retrieving chunk by ID: {e}")
            return None

    async def delete_chunks_by_book_id(self, book_id: str) -> bool:
        """Delete all chunks associated with a specific book ID."""
        try:
            # Find all points with the given book_id
            scroll_result = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="book_id",
                            match=models.MatchValue(value=book_id)
                        )
                    ]
                ),
                limit=10000  # Adjust based on expected max chunks per book
            )

            point_ids = [point.id for point in scroll_result[0]]

            if point_ids:
                self.client.delete(
                    collection_name=self.collection_name,
                    points_selector=models.PointIdsList(
                        points=point_ids
                    )
                )
                logger.info(f"Deleted {len(point_ids)} chunks for book {book_id}")

            return True
        except Exception as e:
            logger.error(f"Error deleting chunks by book ID: {e}")
            return False


# Global instance
qdrant_service = QdrantService()