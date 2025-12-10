import google.generativeai as genai
from src.config import get_config
import logging
from typing import List, Optional, Dict, Any

logger = logging.getLogger(__name__)


class GeminiService:
    def __init__(self):
        config = get_config()
        genai.configure(api_key=config.GEMINI_API_KEY)
        self.model = genai.GenerativeModel(config.GEMINI_MODEL)

    async def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """Generate a response using Gemini based on the prompt and context."""
        try:
            # Construct the full prompt with context
            full_prompt = prompt
            if context:
                full_prompt = f"Based on the following context:\n\n{context}\n\n{prompt}"

            response = await self.model.generate_content_async(full_prompt)

            if response.text:
                return response.text.strip()
            else:
                # If no text was generated, provide a default response
                return "I couldn't generate a response based on the provided information."
        except Exception as e:
            logger.error(f"Error generating response with Gemini: {e}")
            return "I'm sorry, but I encountered an error while processing your request. Please try again."

    async def generate_embedding(self, text: str) -> List[float]:
        """Generate an embedding for the given text using the appropriate service."""
        config = get_config()
        if config.EMBEDDING_MODEL.lower() == "cohere":
            from src.services.cohere_service import cohere_service
            return await cohere_service.generate_embedding(text)
        else:
            # Use Gemini embedding API
            try:
                result = await genai.embed_content_async(
                    model=config.EMBEDDING_MODEL,
                    content=text
                )
                embedding = result['embedding']
                return embedding
            except Exception as e:
                logger.error(f"Error generating embedding with Gemini: {e}")
                # Return a zero vector as fallback
                return [0.0] * 1024  # Assuming 1024-dimensional embeddings for fallback

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a batch of texts using the appropriate service."""
        config = get_config()
        if config.EMBEDDING_MODEL.lower() == "cohere":
            from src.services.cohere_service import cohere_service
            return await cohere_service.generate_embeddings_batch(texts)
        else:
            # Use Gemini embedding API for batch processing
            try:
                result = await genai.embed_content_async(
                    model=config.EMBEDDING_MODEL,
                    content=texts
                )
                embeddings = result['embeddings']
                return embeddings
            except Exception as e:
                logger.error(f"Error generating embeddings batch with Gemini: {e}")
                # Return zero vectors as fallback
                return [[0.0] * 1024 for _ in texts]

    async def validate_answer_relevance(self, question: str, answer: str, context: str) -> bool:
        """Validate if the answer is relevant to the question and context."""
        try:
            validation_prompt = f"""
            You are a validator for a Retrieval-Augmented Generation (RAG) system.
            Your task is to determine if the provided answer is relevant to the question and supported by the context.

            Question: {question}
            Context: {context}
            Answer: {answer}

            Respond with ONLY 'YES' if the answer is relevant and supported by the context, or 'NO' if it's not.
            """

            response = await self.model.generate_content_async(validation_prompt)
            result = response.text.strip().upper()

            return result == "YES"
        except Exception as e:
            logger.error(f"Error validating answer relevance: {e}")
            # Default to True to not block valid responses due to validation errors
            return True


# Global instance
gemini_service = GeminiService()