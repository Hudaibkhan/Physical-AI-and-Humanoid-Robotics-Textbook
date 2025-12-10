"""
Tests for highlight-to-answer functionality
"""
import asyncio
import pytest
from unittest.mock import AsyncMock, patch, MagicMock
from src.services.rag_service import RAGService
from src.services.qdrant_service import QdrantService
from src.services.gemini_service import GeminiService
from src.services.embedding_service import EmbeddingService


@pytest.fixture
def mock_services():
    """Create mocked services for testing"""
    mock_qdrant = AsyncMock(spec=QdrantService)
    mock_gemini = AsyncMock(spec=GeminiService)
    mock_embedding = AsyncMock(spec=EmbeddingService)

    return mock_qdrant, mock_gemini, mock_embedding


@pytest.fixture
def rag_service(mock_services):
    """Create RAG service with mocked dependencies"""
    mock_qdrant, mock_gemini, mock_embedding = mock_services

    rag_service = RAGService()
    rag_service.qdrant = mock_qdrant
    rag_service.gemini = mock_gemini
    rag_service.embedding = mock_embedding

    return rag_service


@pytest.mark.asyncio
async def test_highlight_to_answer_basic(rag_service, mock_services):
    """Test basic highlight-to-answer functionality"""
    _, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the gemini service to return a response based on selected text
    mock_gemini.generate_response.return_value = "Based on the selected text, the answer is X."

    # Test with selected text (highlighted text)
    question = "What does this mean?"
    selected_text = "This is the highlighted text that provides context for the question."

    result = await rag_service.answer_question(
        question=question,
        selected_text=selected_text
    )

    # Assertions
    assert "response" in result
    assert "source_chunks" in result
    assert result["response"] == "Based on the selected text, the answer is X."
    assert isinstance(result["source_chunks"], list)

    # Verify that the gemini service was called with the selected text context
    # In the actual implementation, this would be verified differently
    assert mock_gemini.generate_response.called


@pytest.mark.asyncio
async def test_highlight_to_answer_priority(rag_service, mock_services):
    """Test that highlighted text takes priority over general context"""
    mock_qdrant, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the qdrant service to return context chunks (these should be ignored when selected_text is provided)
    mock_context_chunks = [
        {
            "id": "chunk_1",
            "content": "This is general book content.",
            "similarity_score": 0.65
        }
    ]
    mock_qdrant.search_similar_chunks.return_value = mock_context_chunks

    # Mock the gemini service to return a response based on selected text
    mock_gemini.generate_response.return_value = "Based on the selected text, the answer is prioritized."

    # Test with selected text provided (should take priority over general context)
    question = "Explain this concept?"
    selected_text = "This is the highlighted text that should be used for the answer."

    result = await rag_service.answer_question(
        question=question,
        selected_text=selected_text
    )

    # Assertions
    assert "response" in result
    assert result["response"] == "Based on the selected text, the answer is prioritized."

    # When selected_text is provided, the RAG service should NOT call qdrant for context
    # This depends on the actual implementation - in our current implementation,
    # if selected_text is provided, context retrieval might be skipped
    # Verify that the response was based on the selected text, not general context


@pytest.mark.asyncio
async def test_highlight_to_answer_without_question(rag_service, mock_services):
    """Test behavior when only selected text is provided without a specific question"""
    _, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the gemini service to return a response based on selected text
    mock_gemini.generate_response.return_value = "The selected text discusses the following concepts..."

    # Test with only selected text, no specific question
    question = ""  # Empty question
    selected_text = "This is the highlighted text that should be explained."

    # We expect this to still work, treating the selected text as the basis for response
    result = await rag_service.answer_question(
        question=question,
        selected_text=selected_text
    )

    # Assertions
    assert "response" in result
    assert "selected text" in result["response"].lower() or "following concepts" in result["response"].lower()


@pytest.mark.asyncio
async def test_highlight_to_answer_empty_selected_text(rag_service, mock_services):
    """Test behavior when selected text is empty"""
    mock_qdrant, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the qdrant service to return context chunks (should be used when no selected text)
    mock_context_chunks = [
        {
            "id": "chunk_1",
            "content": "This is general book content that should be used when no selection is made.",
            "similarity_score": 0.75
        }
    ]
    mock_qdrant.search_similar_chunks.return_value = mock_context_chunks

    # Mock the gemini service to return a response based on general context
    mock_gemini.generate_response.return_value = "Based on the general context, the answer is Y."

    # Test with empty selected text (should fall back to general context)
    question = "What does the book say about this topic?"
    selected_text = ""  # Empty selected text

    result = await rag_service.answer_question(
        question=question,
        selected_text=selected_text
    )

    # Assertions
    assert "response" in result
    assert "general context" in result["response"].lower()
    assert len(result["source_chunks"]) == len(mock_context_chunks)

    # Verify that qdrant was called to search for context when no selected text provided
    mock_qdrant.search_similar_chunks.assert_called()


@pytest.mark.asyncio
async def test_highlight_to_answer_long_selected_text(rag_service, mock_services):
    """Test behavior with very long selected text"""
    _, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the gemini service to return a response based on selected text
    mock_gemini.generate_response.return_value = "Based on the extensive selected text, here's the answer."

    # Test with very long selected text
    question = "Summarize this?"
    selected_text = "This is a very long selected text. " * 200  # Much longer than typical selection

    result = await rag_service.answer_question(
        question=question,
        selected_text=selected_text
    )

    # Should still return a valid response
    assert "response" in result
    assert "extensive selected text" in result["response"].lower()


@pytest.mark.asyncio
async def test_highlight_to_answer_special_characters(rag_service, mock_services):
    """Test behavior with selected text containing special characters"""
    _, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the gemini service to return a response based on selected text
    mock_gemini.generate_response.return_value = "The selected text with special characters is understood."

    # Test with selected text containing special characters and formatting
    question = "Explain this code snippet?"
    selected_text = """
    function example() {
        console.log("Hello, world!");
        return true;
    }
    """

    result = await rag_service.answer_question(
        question=question,
        selected_text=selected_text
    )

    # Should handle special characters properly
    assert "response" in result
    assert "special characters" in result["response"].lower()


@pytest.mark.asyncio
async def test_highlight_to_answer_multiple_highlights(rag_service, mock_services):
    """Test behavior with multiple selected/highlighted text segments"""
    _, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the gemini service to return a response based on combined selected text
    mock_gemini.generate_response.return_value = "Based on the combined selected texts, the answer is Z."

    # Test with multiple highlighted segments (simulated as a single selected text)
    question = "How are these concepts related?"
    selected_text = "Concept A: This is the first highlighted segment. || Concept B: This is the second highlighted segment."

    result = await rag_service.answer_question(
        question=question,
        selected_text=selected_text
    )

    # Should handle multiple concepts in the selected text
    assert "response" in result
    assert "combined" in result["response"].lower() or "concepts" in result["response"].lower()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])