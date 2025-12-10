from typing import List, Dict, Optional
from src.services.qdrant_service import qdrant_service
from src.services.gemini_service import gemini_service
from src.services.embedding_service import embedding_service
from src.services.cache_service import cache_service
from src.services.session_service import session_service
import logging

logger = logging.getLogger(__name__)


class RAGService:
    def __init__(self):
        self.qdrant = qdrant_service
        self.gemini = gemini_service
        self.embedding = embedding_service
        self.cache = cache_service
        self.session = session_service

    async def retrieve_context(self, query: str, top_k: int = 5, book_id: Optional[str] = None, similarity_threshold: float = 0.3) -> List[Dict]:
        """Retrieve relevant context chunks based on the query."""
        try:
            # Generate embedding for the query
            query_embedding = await self.embedding.create_embedding(query)

            # Search for similar chunks
            results = await self.qdrant.search_similar_chunks(
                query_embedding=query_embedding,
                top_k=top_k * 2  # Get more results to allow for filtering
            )

            # Filter by similarity threshold and book_id if specified
            filtered_results = []
            for result in results:
                if result["similarity_score"] >= similarity_threshold:
                    if book_id is None or result.get("book_id") == book_id:
                        filtered_results.append(result)

                # Early termination if we have enough high-quality results
                if len(filtered_results) >= top_k:
                    break

            # Return only the top_k results after filtering
            return filtered_results[:top_k]
        except Exception as e:
            logger.error(f"Error retrieving context: {e}")
            return []

    async def generate_answer(self, question: str, context_chunks: List[Dict], selected_text: Optional[str] = None) -> str:
        """Generate an answer based on the question and retrieved context."""
        try:
            # Build context from retrieved chunks
            context_parts = []
            if selected_text:
                # Prioritize selected text if provided
                context_parts.append(f"IMPORTANT CONTEXT FROM USER SELECTION: {selected_text}")
                context_parts.append(f"Question: {question}")
                # Add some retrieved chunks as additional context if available
                for chunk in context_chunks[:2]:  # Only use top 2 chunks to avoid overwhelming
                    context_parts.append(f"Additional context: {chunk['content']}")
            else:
                # Use retrieved chunks as context
                for chunk in context_chunks:
                    context_parts.append(chunk["content"])
                context_parts.append(f"Based on the above information, answer this question: {question}")

            context = "\n\n".join(context_parts)

            # Generate response using Gemini
            if selected_text:
                # When selected text is provided, focus more on that text
                prompt = f"Based on the IMPORTANT CONTEXT FROM USER SELECTION, answer the following question: {question}"
            else:
                prompt = f"Please answer the following question based on the provided context. If the question cannot be answered based on the context, respond with 'This information is not in the book.': {question}"

            response = await self.gemini.generate_response(
                prompt=prompt,
                context=context
            )

            return response
        except Exception as e:
            logger.error(f"Error generating answer: {e}")
            return "I'm sorry, but I encountered an error while processing your request. Please try again."

    async def answer_question(self, question: str, selected_text: Optional[str] = None,
                            session_id: Optional[str] = None, book_id: Optional[str] = None,
                            top_k: int = 5, similarity_threshold: float = 0.3) -> Dict:
        """Complete RAG pipeline: retrieve context and generate answer."""
        try:
            # Generate session ID if not provided
            if not session_id:
                session_id = self.session.generate_session_id()
                await self.session.create_session(session_id)

            # Check cache first for frequently asked questions
            cached_result = await self.cache.get_question_response(question, selected_text or "")
            if cached_result:
                logger.info("Returning cached response for question")

                # Add to session history if session exists
                await self.session.add_message_to_session(session_id, {
                    "role": "user",
                    "content": question,
                    "selected_text": selected_text
                })
                await self.session.add_message_to_session(session_id, {
                    "role": "assistant",
                    "content": cached_result["response"]
                })

                return cached_result

            if selected_text:
                # If selected text is provided, use it as primary context
                context_chunks = []
                answer = await self.generate_answer(question, context_chunks, selected_text)
            else:
                # Retrieve context from the vector database with performance optimizations
                context_chunks = await self.retrieve_context(
                    query=question,
                    top_k=top_k,
                    book_id=book_id,
                    similarity_threshold=similarity_threshold
                )

                # Generate answer based on retrieved context
                answer = await self.generate_answer(question, context_chunks)

            result = {
                "response": answer,
                "source_chunks": [
                    {
                        "id": chunk["id"],
                        "content": chunk["content"][:200] + "..." if len(chunk["content"]) > 200 else chunk["content"],  # Truncate for brevity
                        "similarity_score": chunk["similarity_score"]
                    }
                    for chunk in context_chunks
                ],
                "context_chunks_count": len(context_chunks),
                "confidence": min([chunk["similarity_score"] for chunk in context_chunks]) if context_chunks else 0.0,
                "session_id": session_id
            }

            # Add to session history
            await self.session.add_message_to_session(session_id, {
                "role": "user",
                "content": question,
                "selected_text": selected_text
            })
            await self.session.add_message_to_session(session_id, {
                "role": "assistant",
                "content": answer
            })

            # Cache the result for frequently asked questions (only cache positive responses)
            if "not in the book" not in answer.lower() and len(answer) > 0:
                await self.cache.set_question_response(question, selected_text or "", result)

            return result
        except Exception as e:
            logger.error(f"Error in complete RAG pipeline: {e}")
            return {
                "response": "I'm sorry, but I encountered an error while processing your request. Please try again.",
                "source_chunks": [],
                "context_chunks_count": 0,
                "confidence": 0.0,
                "session_id": session_id
            }

    async def validate_answer(self, question: str, answer: str, context: str) -> bool:
        """Validate if the answer is relevant and based on the context."""
        try:
            is_relevant = await self.gemini.validate_answer_relevance(question, answer, context)
            return is_relevant
        except Exception as e:
            logger.error(f"Error validating answer: {e}")
            return True  # Default to True to not block valid responses


# Global instance
rag_service = RAGService()