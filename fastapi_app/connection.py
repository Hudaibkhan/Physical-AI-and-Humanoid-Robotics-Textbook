from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
import os
from dotenv import load_dotenv
from litellm import completion
import numpy as np
import logging
from agents import AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig
from agents.extensions.models.litellm_model import LitellmModel
# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class ConnectionManager:
    def __init__(self):
        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_url and qdrant_api_key:
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                prefer_grpc=False,  # Using HTTP for compatibility
                timeout=10  # Add timeout for connection
            )
        else:
            logger.warning("QDRANT_URL or QDRANT_API_KEY not set, initializing without remote connection")
            # For local testing, you might want to use local Qdrant
            self.qdrant_client = QdrantClient(
                location=":memory:"  # Use in-memory storage for testing
            )

        self.collection_name = os.getenv("COLLECTION_NAME", "book_chunks")

        # Set vector size based on the embedding model being used
        # For Cohere embeddings, it's 1024 dimensions
        embedding_model = os.getenv("EMBEDDING_MODEL", "cohere")
        if embedding_model == "cohere":
            self.vector_size = 1024  # Cohere embeddings are 1024-dimensional
        else:
            self.vector_size = 3072  # Default for other models like Gemini

    def ensure_collection_exists(self):
        """Ensure the collection exists with proper configuration."""
        try:
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                self.qdrant_client.create_collection(
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
            # Don't raise the exception to allow graceful degradation
            # In production, you might want to handle this differently
            pass

    async def embed(self, text: str) -> List[float]:
        """
        Create embedding for the given text using Cohere API.
        """
        import aiohttp
        import asyncio

        # Get the embedding model from environment
        embedding_model = os.getenv("EMBEDDING_MODEL", "cohere")
        cohere_api_key = os.getenv("COHERE_API_KEY")

        if embedding_model == "cohere" and cohere_api_key:
            # Use Cohere API for embeddings
            url = "https://api.cohere.ai/v1/embed"

            headers = {
                "Authorization": f"Bearer {cohere_api_key}",
                "Content-Type": "application/json"
            }

            data = {
                "texts": [text],
                "model": "embed-english-v3.0",  # Using a standard Cohere embedding model
                "input_type": "search_query"  # Specify this is for search
            }

            try:
                async with aiohttp.ClientSession() as session:
                    async with session.post(url, headers=headers, json=data) as response:
                        if response.status == 200:
                            result = await response.json()
                            embeddings = result.get("embeddings", [])
                            if embeddings and len(embeddings) > 0:
                                return embeddings[0]  # Return the first embedding
                        else:
                            logger.error(f"Cohere API error: {response.status}, {await response.text()}")
            except Exception as e:
                logger.error(f"Error calling Cohere API: {e}")

        # Fallback to a basic hash-based embedding if API fails
        # This is still not ideal for semantic similarity but maintains functionality
        import hashlib
        text_hash = hashlib.md5(text.encode()).hexdigest()
        embedding = [float(ord(c) % 256) / 255.0 for c in text_hash]
        # Pad or truncate to vector_size
        if len(embedding) < self.vector_size:
            embedding.extend([0.0] * (self.vector_size - len(embedding)))
        else:
            embedding = embedding[:self.vector_size]
        return embedding

    async def qdrant_search(self, query_embedding: List[float], top_k: int = 5) -> List[Dict]:
        """Search for similar chunks based on the query embedding."""
        try:
            # Check if the query_points method exists on the client (newer Qdrant API)
            if not hasattr(self.qdrant_client, 'query_points'):
                logger.warning("Qdrant client does not have query_points method - likely not connected properly")
                return []

            # Use the correct collection name - check if it exists
            collection_exists = self.qdrant_client.collection_exists(self.collection_name)
            if not collection_exists:
                # Try the collection name from the debug output
                self.collection_name = "book_chunks"
                collection_exists = self.qdrant_client.collection_exists(self.collection_name)

            if not collection_exists:
                logger.warning(f"Collection {self.collection_name} does not exist")
                return []

            # Use query_points method (newer Qdrant API)
            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k
            )

            results = []
            # The search_results is a QueryResponse object with a 'points' attribute containing ScoredPoint objects
            for hit in search_results.points:  # Access the points attribute
                result = {
                    "id": str(hit.id),
                    "content": hit.payload.get("content", "") if hasattr(hit, 'payload') and hit.payload else "",
                    "similarity_score": hit.score if hasattr(hit, 'score') else 0.0,
                    "book_id": hit.payload.get("book_id") if hasattr(hit, 'payload') and hit.payload else None,
                    "chunk_index": hit.payload.get("chunk_index") if hasattr(hit, 'payload') and hit.payload else None,
                    "metadata": hit.payload.get("metadata", {}) if hasattr(hit, 'payload') and hit.payload else {}
                }
                results.append(result)

            logger.info(f"Found {len(results)} similar chunks")
            return results
        except Exception as e:
            logger.error(f"Error searching similar chunks: {e}")
            # Return empty results but log the error for monitoring
            return []

    async def selected_text_search(self, query: str, selected_text: str, top_k: int = 5) -> List[Dict]:
        """
        Process selected text, create embeddings, and find matches with the query.
        This implements the selected text RAG functionality.
        """
        try:
            # Validate selected text length to prevent token limit issues
            if len(selected_text) > 10000:  # Adjust this limit as needed
                logger.warning(f"Selected text is very long ({len(selected_text)} chars), truncating for performance")
                # Truncate to a reasonable length while preserving sentence boundaries
                selected_text = self._truncate_selected_text(selected_text, max_length=10000)

            # For this implementation, we'll split the selected text into chunks
            # and compute similarity between the query and these chunks
            import re

            # Split the selected text into smaller chunks (e.g., sentences or paragraphs)
            # This is a simple approach - in a real implementation, you might want more sophisticated chunking
            sentences = re.split(r'[.!?]+', selected_text)
            chunks = []
            for i, sentence in enumerate(sentences):
                if sentence.strip():
                    chunks.append({
                        "id": f"selected_chunk_{i}",
                        "content": sentence.strip(),
                        "chunk_index": i
                    })

            # Create embeddings for the query
            query_embedding = await self.embed(query)

            # Create embeddings for each chunk and calculate similarity
            similarities = []
            for chunk in chunks:
                chunk_embedding = await self.embed(chunk["content"])
                # Calculate cosine similarity
                similarity = self.cosine_similarity(query_embedding, chunk_embedding)
                similarities.append({
                    "id": chunk["id"],
                    "content": chunk["content"],
                    "similarity_score": similarity,
                    "chunk_index": chunk["chunk_index"]
                })

            # Sort by similarity score and return top_k
            similarities.sort(key=lambda x: x["similarity_score"], reverse=True)
            return similarities[:top_k]

        except Exception as e:
            logger.error(f"Error in selected text search: {e}")
            return []

    async def direct_rag_search(self, query: str, mode: str, top_k: int = 5, selected_text: str = None) -> List[Dict]:
        """
        Direct RAG search function that can be called directly (not as a function tool).
        This is used by the API to get search results without going through the agent's function tool.
        """
        try:
            if mode == "rag":
                # Convert user query to embedding
                query_embedding = await self.embed(query)

                # Retrieve from Qdrant
                results = await self.qdrant_search(query_embedding, top_k)
            elif mode == "selected":
                if not selected_text:
                    logger.warning("Selected text mode called without selected_text")
                    return []

                # Retrieve from selected text
                results = await self.selected_text_search(query, selected_text, top_k)
            else:
                raise ValueError(f"Invalid mode: {mode}. Must be 'rag' or 'selected'")

            return results
        except Exception as e:
            logger.error(f"Error in direct_rag_search: {e}")
            return []

    def _truncate_selected_text(self, text: str, max_length: int = 10000) -> str:
        """
        Truncate selected text to a maximum length while preserving sentence boundaries.
        """
        if len(text) <= max_length:
            return text

        # Try to find a sentence boundary near the max length
        truncated = text[:max_length]

        # Look for the last sentence ending before max_length
        for delimiter in ['.', '!', '?', '\n']:
            last_idx = truncated.rfind(delimiter)
            if last_idx != -1:
                return text[:last_idx + 1]

        # If no sentence boundary found, just truncate at max_length
        return text[:max_length]

    def cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """Calculate cosine similarity between two vectors."""
        # Convert to numpy arrays
        v1 = np.array(vec1)
        v2 = np.array(vec2)

        # Calculate cosine similarity: (A · B) / (||A|| * ||B||)
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        if norm_v1 == 0 or norm_v2 == 0:
            return 0.0

        return float(dot_product / (norm_v1 * norm_v2))

class SessionManager:
    def __init__(self):
        # In a real implementation, this would connect to a persistent store like Redis or a database
        # For this implementation, we'll use an in-memory dictionary
        # This is not suitable for production use
        self.sessions = {}

    def create_session(self, session_id: str = None):
        """Create a new session with a unique ID if not provided."""
        import uuid
        if not session_id:
            session_id = str(uuid.uuid4())

        self.sessions[session_id] = {
            "id": session_id,
            "created_at": self.get_current_timestamp(),
            "last_activity": self.get_current_timestamp(),
            "conversation_history": []
        }
        return session_id

    def add_message_to_session(self, session_id: str, message: dict):
        """Add a message to the conversation history in a session."""
        if session_id not in self.sessions:
            self.create_session(session_id)

        self.sessions[session_id]["conversation_history"].append(message)
        self.sessions[session_id]["last_activity"] = self.get_current_timestamp()

    def get_session_history(self, session_id: str):
        """Get the conversation history for a session."""
        if session_id in self.sessions:
            return self.sessions[session_id]["conversation_history"]
        return []

    def get_current_timestamp(self):
        """Get the current timestamp."""
        from datetime import datetime
        return datetime.utcnow().isoformat()

# Global instances

connection_manager = ConnectionManager()
session_manager = SessionManager()




load_dotenv()
gemini_api_key = os.getenv("GEMINI_API_KEY")
# Check if the API key is present; if not, set up for simulated responses
if not gemini_api_key:
    print("WARNING: GEMINI_API_KEY is not set. The system will run in simulation mode without external API access.")
    # In simulation mode, we'll handle this in the agent implementation
    external_client = None
    model = None
    config = None
else:
    #Reference: https://ai.google.dev/gemini-api/docs/openai
    external_client = AsyncOpenAI( api_key=gemini_api_key,
                                  base_url="https://generativelanguage.googleapis.com/v1beta/openai/", )

model = OpenAIChatCompletionsModel( model="gemini-2.5-flash", openai_client=external_client )

config = RunConfig( model=model, model_provider=external_client, tracing_disabled=True )