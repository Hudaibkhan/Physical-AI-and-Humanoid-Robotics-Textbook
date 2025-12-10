import cohere
from src.config import get_config
import logging
from typing import List, Optional

logger = logging.getLogger(__name__)


class CohereService:
    def __init__(self):
        config = get_config()
        self.client = cohere.Client(config.COHERE_API_KEY)

    async def generate_embedding(self, text: str) -> List[float]:
        """Generate an embedding for the given text using Cohere."""
        try:
            response = self.client.embed(
                texts=[text],
                model='embed-english-v3.0',  # Using Cohere's latest embedding model
                input_type="search_document"  # Optimize for search use case
            )
            embedding = response.embeddings[0]  # Get the first (and only) embedding
            return embedding
        except Exception as e:
            logger.error(f"Error generating embedding with Cohere: {e}")
            # Return a zero vector as fallback
            return [0.0] * 1024  # Cohere embeddings are typically 1024-dimensional

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a batch of texts using Cohere."""
        try:
            response = self.client.embed(
                texts=texts,
                model='embed-english-v3.0',  # Using Cohere's latest embedding model
                input_type="search_document"  # Optimize for search use case
            )
            embeddings = response.embeddings
            return embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings batch with Cohere: {e}")
            # Return zero vectors as fallback
            return [[0.0] * 1024 for _ in texts]


# Global instance
cohere_service = CohereService()