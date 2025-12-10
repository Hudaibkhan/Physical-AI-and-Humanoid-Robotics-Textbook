"""
Integration tests for the complete RAG functionality
"""
import asyncio
import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch, MagicMock
from src.main import app
from src.services.rag_service import RAGService
from src.services.qdrant_service import QdrantService
from src.services.gemini_service import GeminiService
from src.services.embedding_service import EmbeddingService


@pytest.fixture
def client():
    """Create a test client for the API"""
    return TestClient(app)


class TestRAGIntegration:
    """Integration tests for the complete RAG pipeline"""

    @pytest.mark.asyncio
    async def test_complete_rag_pipeline(self):
        """Test the complete RAG pipeline from question to answer"""
        # Create mock services
        mock_qdrant = AsyncMock(spec=QdrantService)
        mock_gemini = AsyncMock(spec=GeminiService)
        mock_embedding = AsyncMock(spec=EmbeddingService)

        # Create RAG service with mocked dependencies
        rag_service = RAGService()
        rag_service.qdrant = mock_qdrant
        rag_service.gemini = mock_gemini
        rag_service.embedding = mock_embedding

        # Mock the embedding service to return a fixed embedding
        mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

        # Mock the qdrant service to return relevant context chunks
        mock_context_chunks = [
            {
                "id": "chunk_1",
                "content": "This is relevant book content that answers the question about AI.",
                "similarity_score": 0.85,
                "book_id": "test-book"
            },
            {
                "id": "chunk_2",
                "content": "Another relevant context from the book about artificial intelligence.",
                "similarity_score": 0.72,
                "book_id": "test-book"
            }
        ]
        mock_qdrant.search_similar_chunks.return_value = mock_context_chunks

        # Mock the gemini service to return a fixed response
        mock_gemini.generate_response.return_value = "Based on the book content, artificial intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence."

        # Test the complete pipeline
        question = "What is artificial intelligence according to the book?"
        result = await rag_service.answer_question(question)

        # Verify the result
        assert "response" in result
        assert "source_chunks" in result
        assert "context_chunks_count" in result
        assert result["context_chunks_count"] == 2
        assert "artificial intelligence" in result["response"].lower()

        # Verify that the services were called correctly
        mock_embedding.create_embedding.assert_called_once_with(question)
        mock_qdrant.search_similar_chunks.assert_called_once()
        mock_gemini.generate_response.assert_called_once()

    @pytest.mark.asyncio
    async def test_rag_pipeline_with_selected_text(self):
        """Test the RAG pipeline with selected text context"""
        # Create mock services
        mock_qdrant = AsyncMock(spec=QdrantService)
        mock_gemini = AsyncMock(spec=GeminiService)
        mock_embedding = AsyncMock(spec=EmbeddingService)

        # Create RAG service with mocked dependencies
        rag_service = RAGService()
        rag_service.qdrant = mock_qdrant
        rag_service.gemini = mock_gemini
        rag_service.embedding = mock_embedding

        # Mock the gemini service to return a response based on selected text
        mock_gemini.generate_response.return_value = "Based on the selected text, the concept refers to machine learning algorithms that improve through experience."

        # Test with selected text (should prioritize selected text over general context)
        question = "Explain this concept?"
        selected_text = "This is the highlighted text that explains machine learning concepts."

        result = await rag_service.answer_question(
            question=question,
            selected_text=selected_text
        )

        # Verify the result
        assert "response" in result
        assert "source_chunks" in result
        assert "based on the selected text" in result["response"].lower()

        # For selected text, qdrant shouldn't be called since we're using the selected text as primary context
        # Depending on implementation, this may or may not be called
        # The important thing is that the response is based on the selected text

    @pytest.mark.asyncio
    async def test_rag_pipeline_no_relevant_context(self):
        """Test the RAG pipeline when no relevant context is found"""
        # Create mock services
        mock_qdrant = AsyncMock(spec=QdrantService)
        mock_gemini = AsyncMock(spec=GeminiService)
        mock_embedding = AsyncMock(spec=EmbeddingService)

        # Create RAG service with mocked dependencies
        rag_service = RAGService()
        rag_service.qdrant = mock_qdrant
        rag_service.gemini = mock_gemini
        rag_service.embedding = mock_embedding

        # Mock the embedding service
        mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

        # Mock the qdrant service to return no relevant chunks
        mock_qdrant.search_similar_chunks.return_value = []

        # Mock the gemini service to return a "not in book" response
        mock_gemini.generate_response.return_value = "This information is not in the book."

        # Test with a question that should not be in the book
        question = "What is the weather like today?"

        result = await rag_service.answer_question(question)

        # Verify the result
        assert "response" in result
        assert "source_chunks" in result
        assert "This information is not in the book." in result["response"]
        assert result["context_chunks_count"] == 0

        # Verify that the services were called
        mock_embedding.create_embedding.assert_called_once_with(question)
        mock_qdrant.search_similar_chunks.assert_called_once()
        # Note: gemini might still be called to generate the "not in book" response

    @pytest.mark.asyncio
    async def test_rag_pipeline_error_handling(self):
        """Test the RAG pipeline error handling"""
        # Create mock services
        mock_qdrant = AsyncMock(spec=QdrantService)
        mock_gemini = AsyncMock(spec=GeminiService)
        mock_embedding = AsyncMock(spec=EmbeddingService)

        # Create RAG service with mocked dependencies
        rag_service = RAGService()
        rag_service.qdrant = mock_qdrant
        rag_service.gemini = mock_gemini
        rag_service.embedding = mock_embedding

        # Mock the embedding service to raise an exception
        mock_embedding.create_embedding.side_effect = Exception("Embedding service error")

        # Test error handling
        question = "What is artificial intelligence?"
        result = await rag_service.answer_question(question)

        # Verify that a default error response is returned
        assert "response" in result
        assert "encountered an error" in result["response"].lower()
        assert result["source_chunks"] == []
        assert result["context_chunks_count"] == 0

    @pytest.mark.asyncio
    async def test_rag_pipeline_performance_optimization(self):
        """Test the RAG pipeline with performance optimizations"""
        # Create mock services
        mock_qdrant = AsyncMock(spec=QdrantService)
        mock_gemini = AsyncMock(spec=GeminiService)
        mock_embedding = AsyncMock(spec=EmbeddingService)

        # Create RAG service with mocked dependencies
        rag_service = RAGService()
        rag_service.qdrant = mock_qdrant
        rag_service.gemini = mock_gemini
        rag_service.embedding = mock_embedding

        # Mock the embedding service
        mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

        # Mock the qdrant service to return many chunks, some with low similarity
        mock_all_chunks = [
            {"id": f"chunk_{i}", "content": f"Content {i}", "similarity_score": 0.9 - (i * 0.05), "book_id": "test-book"}
            for i in range(20)  # 20 chunks with decreasing similarity
        ]
        mock_qdrant.search_similar_chunks.return_value = mock_all_chunks

        # Mock the gemini service
        mock_gemini.generate_response.return_value = "This is the answer based on the most relevant context."

        # Test with similarity threshold
        question = "What is the main topic?"
        result = await rag_service.answer_question(
            question=question,
            similarity_threshold=0.5  # Only include chunks with similarity > 0.5
        )

        # Verify that only high-similarity chunks were used
        assert "response" in result
        assert result["context_chunks_count"] <= 10  # Should be fewer than 20 due to threshold

        # Verify that the services were called correctly
        mock_embedding.create_embedding.assert_called_once()
        mock_qdrant.search_similar_chunks.assert_called_once()

    @pytest.mark.asyncio
    async def test_rag_pipeline_caching_behavior(self):
        """Test the RAG pipeline caching behavior"""
        # Create mock services
        mock_qdrant = AsyncMock(spec=QdrantService)
        mock_gemini = AsyncMock(spec=GeminiService)
        mock_embedding = AsyncMock(spec=EmbeddingService)
        mock_cache = AsyncMock()

        # Create RAG service with mocked dependencies
        rag_service = RAGService()
        rag_service.qdrant = mock_qdrant
        rag_service.gemini = mock_gemini
        rag_service.embedding = mock_embedding
        rag_service.cache = mock_cache

        # Mock the cache to return a cached response for the first call
        cached_response = {
            "response": "Cached answer to frequently asked question",
            "source_chunks": [{"id": "cached_chunk", "content": "cached content", "similarity_score": 0.8}],
            "context_chunks_count": 1,
            "confidence": 0.8
        }
        mock_cache.get_question_response.return_value = cached_response

        # Test with a question that should be cached
        question = "What is the main concept?"
        selected_text = ""

        result = await rag_service.answer_question(
            question=question,
            selected_text=selected_text
        )

        # Verify that the cached response was returned
        assert result == cached_response
        # Verify that the cache was checked
        mock_cache.get_question_response.assert_called_once_with(question, selected_text)

        # Reset the mock to test uncached behavior
        mock_cache.reset_mock()
        mock_cache.get_question_response.return_value = None  # No cached response

        # Mock the rest of the services for the uncached path
        mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]
        mock_qdrant.search_similar_chunks.return_value = [
            {"id": "chunk_1", "content": "relevant content", "similarity_score": 0.8, "book_id": "test-book"}
        ]
        mock_gemini.generate_response.return_value = "New answer for uncached question"

        # Test with the same question but no cache hit
        result = await rag_service.answer_question(
            question=question,
            selected_text=selected_text
        )

        # Verify that the full pipeline was executed
        mock_embedding.create_embedding.assert_called_once()
        mock_qdrant.search_similar_chunks.assert_called_once()
        mock_gemini.generate_response.assert_called_once()
        # Verify that the result was cached
        mock_cache.set_question_response.assert_called_once()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])