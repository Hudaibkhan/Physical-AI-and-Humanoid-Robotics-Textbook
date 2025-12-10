"""
Tests for basic question-answering functionality with book content
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
async def test_basic_question_answering(rag_service, mock_services):
    """Test basic question-answering functionality with book content"""
    _, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service to return a fixed embedding
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the gemini service to return a fixed response
    mock_gemini.generate_response.return_value = "This is a test answer based on the book content."

    # Test the answer_question method
    question = "What is the main concept discussed in the book?"
    result = await rag_service.answer_question(question)

    # Assertions
    assert "response" in result
    assert "source_chunks" in result
    assert "context_chunks_count" in result
    assert result["response"] == "This is a test answer based on the book content."
    assert isinstance(result["source_chunks"], list)
    assert isinstance(result["context_chunks_count"], int)

    # Verify that the embedding service was called with the question
    mock_embedding.create_embedding.assert_called_once_with(question)

    # Verify that the gemini service was called to generate the response
    mock_gemini.generate_response.assert_called_once()


@pytest.mark.asyncio
async def test_question_answering_with_selected_text(rag_service, mock_services):
    """Test question-answering functionality with selected text context"""
    _, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the gemini service to return a fixed response
    mock_gemini.generate_response.return_value = "Based on the selected text, the answer is X."

    # Test with selected text
    question = "Explain this concept?"
    selected_text = "This is the selected text that provides context for the question."
    result = await rag_service.answer_question(question, selected_text=selected_text)

    # Assertions
    assert "response" in result
    assert "source_chunks" in result
    assert result["response"] == "Based on the selected text, the answer is X."

    # Verify that gemini was called with the selected text as primary context
    args, kwargs = mock_gemini.generate_response.call_args
    assert "selected_text" in kwargs or selected_text in str(args)


@pytest.mark.asyncio
async def test_out_of_book_question_handling(rag_service, mock_services):
    """Test handling of questions that are not in the book"""
    _, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the gemini service to return a "not in book" response
    mock_gemini.generate_response.return_value = "This information is not in the book."

    # Test with a question that should not be in the book
    question = "What is the weather like today?"
    result = await rag_service.answer_question(question)

    # Assertions
    assert "response" in result
    assert "This information is not in the book." in result["response"]

    # Verify that the service was called
    mock_embedding.create_embedding.assert_called_once_with(question)
    mock_gemini.generate_response.assert_called_once()


@pytest.mark.asyncio
async def test_relevant_context_retrieval(rag_service, mock_services):
    """Test that relevant context is retrieved for questions"""
    mock_qdrant, mock_gemini, mock_embedding = mock_services

    # Mock the embedding service
    mock_embedding.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

    # Mock the qdrant service to return relevant context chunks
    mock_context_chunks = [
        {
            "id": "chunk_1",
            "content": "This is relevant book content that answers the question.",
            "similarity_score": 0.85
        },
        {
            "id": "chunk_2",
            "content": "This is additional context from the book.",
            "similarity_score": 0.72
        }
    ]
    mock_qdrant.search_similar_chunks.return_value = mock_context_chunks

    # Mock the gemini service to return a response based on context
    mock_gemini.generate_response.return_value = "Based on the book content, the answer is Y."

    # Test question answering with context retrieval
    question = "What does the book say about topic X?"
    result = await rag_service.answer_question(question)

    # Assertions
    assert "response" in result
    assert len(result["source_chunks"]) == len(mock_context_chunks)
    assert result["context_chunks_count"] == len(mock_context_chunks)

    # Verify that qdrant was called to search for similar chunks
    mock_qdrant.search_similar_chunks.assert_called_once()

    # Verify that the context chunks were properly formatted
    for i, chunk in enumerate(result["source_chunks"]):
        assert chunk["id"] == mock_context_chunks[i]["id"]
        assert chunk["content"] in mock_context_chunks[i]["content"]
        assert chunk["similarity_score"] == mock_context_chunks[i]["similarity_score"]


@pytest.mark.asyncio
async def test_empty_question_handling(rag_service):
    """Test handling of empty or invalid questions"""
    # Test with empty question
    with pytest.raises(Exception):
        await rag_service.answer_question("")

    # Test with whitespace-only question
    with pytest.raises(Exception):
        await rag_service.answer_question("   ")


@pytest.mark.asyncio
async def test_long_question_handling(rag_service, mock_services):
    """Test handling of very long questions"""
    _, mock_gemini, mock_embedding = mock_services

    # Mock services
    mock_embedding.create_embedding.return_value = [0.1] * 768  # Typical embedding size
    mock_gemini.generate_response.return_value = "This is the answer to your long question."

    # Very long question
    long_question = "This is a very long question. " * 100

    result = await rag_service.answer_question(long_question)

    # Should still return a valid response
    assert "response" in result
    assert len(result["response"]) > 0

    # Verify that embedding was called (may be called multiple times depending on implementation)
    assert mock_embedding.create_embedding.called


if __name__ == "__main__":
    pytest.main([__file__, "-v"])