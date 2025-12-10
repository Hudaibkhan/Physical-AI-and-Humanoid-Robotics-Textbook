from typing import Dict, Any, List, Optional
import logging
from pydantic import BaseModel
from openai import OpenAI
from ..utils.gemini_provider import get_gemini_provider
from ..tools.retrieve_chunks import retrieve_chunks_tool
from ..tools.answer_with_context import answer_with_context_tool
from ..config import get_config

logger = logging.getLogger(__name__)

class RAGAgent:
    """
    RAG Agent that uses OpenAI Agent SDK with custom tools and Gemini as the LLM provider
    """
    def __init__(self):
        self.gemini_provider = get_gemini_provider()
        self.config = get_config()
        self.logger = logging.getLogger(self.__class__.__name__)

    async def process_query(self, query: str, max_chunks: Optional[int] = None) -> Dict[str, Any]:
        """
        Process a user query using RAG approach

        Args:
            query: The user's query
            max_chunks: Maximum number of chunks to retrieve (optional)

        Returns:
            Dictionary containing the answer and sources
        """
        try:
            # Step 1: Retrieve relevant chunks using the retrieve_chunks tool
            retrieve_result = await retrieve_chunks_tool.execute(query=query, limit=max_chunks)

            if not retrieve_result.success:
                return {
                    "answer": "I'm sorry, I encountered an error while retrieving information. Please try again later.",
                    "sources": [],
                    "error": retrieve_result.error
                }

            chunks = retrieve_result.data

            # Check if we found any relevant chunks
            if not chunks:
                return {
                    "answer": "I couldn't find any relevant information to answer your question. Please try rephrasing or check if the embeddings have been generated.",
                    "sources": [],
                    "error": "No relevant chunks found"
                }

            # Step 2: Generate answer using the answer_with_context tool
            answer_result = await answer_with_context_tool.execute(question=query, chunks=chunks)

            if not answer_result.success:
                return {
                    "answer": "I'm sorry, I encountered an error while generating an answer. Please try again later.",
                    "sources": [chunk["id"] for chunk in chunks],
                    "error": answer_result.error
                }

            answer = answer_result.data

            # Extract sources from chunks
            sources = [chunk["id"] for chunk in chunks]

            return {
                "answer": answer,
                "sources": sources,
                "confidence": min(1.0, len(chunks) / 5.0),  # Simple confidence based on number of chunks found
                "chunks_used": len(chunks)
            }

        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}")
            return {
                "answer": "I'm sorry, I encountered an unexpected error while processing your query. Please try again later.",
                "sources": [],
                "error": str(e)
            }

    async def process_selected_text_query(self, selected_text: str, query: str) -> Dict[str, Any]:
        """
        Process a query based only on selected text (bypassing Qdrant)

        Args:
            selected_text: The text selected by the user
            query: The user's query about the selected text

        Returns:
            Dictionary containing the answer based only on the selected text
        """
        try:
            # Create a chunk from the selected text
            chunks = [{
                "id": "selected_text_chunk",
                "content": selected_text,
                "source_file": "selected_text",
                "chunk_index": 0,
                "score": 1.0,  # Highest possible score
                "metadata": {}
            }]

            # Generate answer using the answer_with_context tool
            answer_result = await answer_with_context_tool.execute(question=query, chunks=chunks)

            if not answer_result.success:
                return {
                    "answer": "I'm sorry, I encountered an error while generating an answer from the selected text. Please try again later.",
                    "sources": [],
                    "error": answer_result.error
                }

            answer = answer_result.data

            return {
                "answer": answer,
                "sources": ["selected_text_chunk"],
                "confidence": 1.0,  # High confidence since we're using exact selected text
                "chunks_used": 1
            }

        except Exception as e:
            self.logger.error(f"Error processing selected text query: {str(e)}")
            return {
                "answer": "I'm sorry, I encountered an unexpected error while processing your selected text query. Please try again later.",
                "sources": [],
                "error": str(e)
            }

# Global instance for reuse
rag_agent = RAGAgent()

def get_rag_agent() -> RAGAgent:
    """
    Get the RAG agent instance
    """
    return rag_agent