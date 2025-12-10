import cohere
from typing import List, Optional
import os
from dotenv import load_dotenv
import logging
from pydantic import BaseModel

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class EmbeddingUtils:
    """
    Utility class for generating embeddings using Cohere
    """
    def __init__(self):
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY must be set in environment variables")

        self.client = cohere.Client(cohere_api_key)

    def embed_text(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        try:
            response = self.client.embed(
                texts=texts,
                model="embed-multilingual-v3.0",  # Using multilingual model for broader language support
                input_type="search_document"  # Optimize for search tasks
            )

            embeddings = [embedding for embedding in response.embeddings]
            logger.info(f"Generated embeddings for {len(texts)} text(s)")

            return embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise

    def embed_single_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text

        Args:
            text: Text string to embed

        Returns:
            Embedding vector as a list of floats
        """
        return self.embed_text([text])[0]

# Global instance for reuse
embedding_utils = EmbeddingUtils()

def get_embedding_utils() -> EmbeddingUtils:
    """
    Get the embedding utilities instance
    """
    return embedding_utils