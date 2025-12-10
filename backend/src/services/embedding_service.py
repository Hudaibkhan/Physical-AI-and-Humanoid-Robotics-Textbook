from typing import List
from src.services.gemini_service import gemini_service
import logging

logger = logging.getLogger(__name__)


class EmbeddingService:
    def __init__(self):
        pass

    async def create_embedding(self, text: str) -> List[float]:
        """Create an embedding for the given text using the configured embedding model."""
        try:
            embedding = await gemini_service.generate_embedding(text)
            return embedding
        except Exception as e:
            logger.error(f"Error creating embedding: {e}")
            # Return a zero vector as fallback
            return [0.0] * 768

    async def create_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Create embeddings for a batch of texts."""
        try:
            embeddings = await gemini_service.generate_embeddings_batch(texts)
            return embeddings
        except Exception as e:
            logger.error(f"Error creating embeddings batch: {e}")
            # Return zero vectors as fallback
            return [[0.0] * 768 for _ in texts]

    async def calculate_similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """Calculate cosine similarity between two embeddings."""
        try:
            # Calculate dot product
            dot_product = sum(a * b for a, b in zip(embedding1, embedding2))

            # Calculate magnitudes
            magnitude1 = sum(a * a for a in embedding1) ** 0.5
            magnitude2 = sum(b * b for b in embedding2) ** 0.5

            # Calculate cosine similarity
            if magnitude1 == 0 or magnitude2 == 0:
                return 0.0

            similarity = dot_product / (magnitude1 * magnitude2)
            return similarity
        except Exception as e:
            logger.error(f"Error calculating similarity: {e}")
            return 0.0


# Global instance
embedding_service = EmbeddingService()