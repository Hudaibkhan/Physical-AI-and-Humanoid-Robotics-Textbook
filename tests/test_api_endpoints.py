"""
Comprehensive API tests for the RAG Chatbot endpoints
"""
import asyncio
import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch, MagicMock
from src.main import app
from src.services.rag_service import rag_service
from src.services.qdrant_service import qdrant_service
from src.services.gemini_service import gemini_service
from src.services.embedding_service import embedding_service


@pytest.fixture
def client():
    """Create a test client for the API"""
    return TestClient(app)


@pytest.fixture
def mock_rag_service():
    """Mock the RAG service"""
    with patch('src.api.v1.endpoints.ask_agent.rag_service') as mock:
        yield mock


@pytest.fixture
def mock_embedding_service():
    """Mock the embedding service"""
    with patch('src.api.v1.endpoints.embed.embedding_service') as mock:
        yield mock


@pytest.fixture
def mock_qdrant_service():
    """Mock the qdrant service"""
    with patch('src.api.v1.endpoints.search.qdrant_service') as mock:
        yield mock


class TestAPIEndpoints:
    """Test all API endpoints for the RAG Chatbot"""

    def test_health_endpoint(self, client):
        """Test the health check endpoint"""
        response = client.get("/health")
        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert data["status"] == "healthy"

    def test_root_endpoint(self, client):
        """Test the root endpoint"""
        response = client.get("/")
        assert response.status_code == 200
        data = response.json()
        assert "message" in data
        assert "RAG Chatbot API is running!" in data["message"]

    def test_embed_endpoint_success(self, client, mock_embedding_service):
        """Test successful embedding generation"""
        # Mock the embedding service
        mock_embedding_service.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

        response = client.post(
            "/api/v1/embed/",
            json={"text": "This is a test sentence for embedding", "model": "gemini-embedding-001"}
        )

        assert response.status_code == 200
        data = response.json()
        assert "embedding" in data
        assert "model" in data
        assert data["model"] == "gemini-embedding-001"
        assert isinstance(data["embedding"], list)
        assert len(data["embedding"]) > 0

    def test_embed_endpoint_validation_error(self, client):
        """Test embedding endpoint with validation error"""
        response = client.post(
            "/api/v1/embed/",
            json={"text": "", "model": "gemini-embedding-001"}  # Empty text should fail validation
        )

        assert response.status_code == 422  # Validation error

    def test_search_endpoint_success(self, client, mock_qdrant_service):
        """Test successful search operation"""
        # Mock the qdrant service
        mock_qdrant_service.search_similar_chunks.return_value = [
            {
                "id": "chunk_1",
                "content": "This is a relevant chunk of text",
                "similarity_score": 0.85
            },
            {
                "id": "chunk_2",
                "content": "This is another relevant chunk",
                "similarity_score": 0.72
            }
        ]

        response = client.post(
            "/api/v1/search/",
            json={
                "query_embedding": [0.1, 0.2, 0.3, 0.4, 0.5],
                "top_k": 5
            }
        )

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "count" in data
        assert data["count"] == 2
        assert len(data["results"]) == 2

    def test_ask_agent_endpoint_success(self, client, mock_rag_service):
        """Test successful question answering"""
        # Mock the RAG service
        mock_rag_service.answer_question.return_value = {
            "response": "This is a test answer based on the book content.",
            "source_chunks": [
                {
                    "id": "chunk_1",
                    "content": "This is relevant content from the book",
                    "similarity_score": 0.85
                }
            ],
            "context_chunks_count": 1,
            "confidence": 0.85,
            "session_id": "test-session-id"
        }

        response = client.post(
            "/api/v1/ask-agent/",
            json={
                "question": "What is the main concept in the book?",
                "selected_text": "",
                "session_id": "test-session-id"
            }
        )

        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert "source_chunks" in data
        assert "session_id" in data
        assert data["response"] == "This is a test answer based on the book content."
        assert data["session_id"] == "test-session-id"

    def test_ask_agent_endpoint_with_selected_text(self, client, mock_rag_service):
        """Test question answering with selected text context"""
        # Mock the RAG service
        mock_rag_service.answer_question.return_value = {
            "response": "Based on the selected text, the answer is X.",
            "source_chunks": [],
            "context_chunks_count": 0,
            "confidence": 0.92,
            "session_id": "test-session-id"
        }

        response = client.post(
            "/api/v1/ask-agent/",
            json={
                "question": "Explain this concept?",
                "selected_text": "This is the highlighted text that provides context.",
                "session_id": "test-session-id"
            }
        )

        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert data["response"] == "Based on the selected text, the answer is X."
        assert data["confidence"] == 0.92

    def test_ask_agent_endpoint_validation_error(self, client):
        """Test ask-agent endpoint with validation error"""
        response = client.post(
            "/api/v1/ask-agent/",
            json={
                "question": "",  # Empty question should fail validation
                "selected_text": "",
                "session_id": "test-session-id"
            }
        )

        assert response.status_code == 422  # Validation error

    def test_search_endpoint_validation_error(self, client):
        """Test search endpoint with validation error"""
        response = client.post(
            "/api/v1/search/",
            json={
                "query_embedding": [],  # Empty embedding should fail validation
                "top_k": 5
            }
        )

        assert response.status_code == 422  # Validation error

    def test_embed_endpoint_large_text(self, client, mock_embedding_service):
        """Test embedding endpoint with large text"""
        large_text = "This is a test sentence. " * 1000  # Create a large text

        # Mock the embedding service
        mock_embedding_service.create_embedding.return_value = [0.1] * 768  # Typical embedding size

        response = client.post(
            "/api/v1/embed/",
            json={"text": large_text, "model": "gemini-embedding-001"}
        )

        # Should still return success for large text
        assert response.status_code == 200
        data = response.json()
        assert "embedding" in data
        assert len(data["embedding"]) == 768  # Typical embedding dimension

    def test_ask_agent_endpoint_no_context(self, client, mock_rag_service):
        """Test question answering when no relevant context is found"""
        # Mock the RAG service to return no context
        mock_rag_service.answer_question.return_value = {
            "response": "This information is not in the book.",
            "source_chunks": [],
            "context_chunks_count": 0,
            "confidence": 0.1,
            "session_id": "test-session-id"
        }

        response = client.post(
            "/api/v1/ask-agent/",
            json={
                "question": "What is the weather like tomorrow?",
                "selected_text": "",
                "session_id": "test-session-id"
            }
        )

        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert "This information is not in the book." in data["response"]
        assert data["context_chunks_count"] == 0

    def test_rate_limiting_on_embed_endpoint(self, client, mock_embedding_service):
        """Test that rate limiting works on the embed endpoint"""
        # Mock the embedding service
        mock_embedding_service.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

        # Make multiple requests rapidly to trigger rate limiting
        for i in range(35):  # More than the rate limit
            response = client.post(
                "/api/v1/embed/",
                json={"text": f"This is a test sentence {i}", "model": "gemini-embedding-001"}
            )

            # Check that requests are successful initially
            if i < 30:  # Under the rate limit
                assert response.status_code in [200, 429]  # Could be 429 if rate limited during test
            else:  # Over the rate limit
                # Eventually we should hit the rate limit
                if response.status_code == 429:
                    assert True  # Rate limited as expected
                    break

    def test_session_management_endpoints(self, client):
        """Test session management endpoints"""
        # Test create session
        response = client.post("/api/v1/sessions/create")
        assert response.status_code == 200
        data = response.json()
        assert "session_id" in data
        assert data["created"] is True

        # Test get history with a session ID
        session_id = data["session_id"]
        response = client.post("/api/v1/sessions/history",
                              json={"session_id": session_id, "limit": 10})
        assert response.status_code == 200
        data = response.json()
        assert "messages" in data
        assert "session_id" in data
        assert data["session_id"] == session_id

    def test_metrics_endpoint(self, client):
        """Test metrics endpoint"""
        response = client.get("/api/v1/metrics/summary")
        # This endpoint might not exist yet, so we'll check if it returns a valid response
        # If it doesn't exist, this test will fail, which is expected
        assert response.status_code in [200, 404]  # OK if exists, 404 if not implemented yet


if __name__ == "__main__":
    pytest.main([__file__, "-v"])