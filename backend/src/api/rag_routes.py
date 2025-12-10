from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
import logging
from pydantic import BaseModel
from ..agents.rag_agent import get_rag_agent
from ..utils.error_handlers import embedding_not_found_error
from ..utils.qdrant_client import get_qdrant_client
from ..config import get_config

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/rag", tags=["rag"])

class RAGQueryRequest(BaseModel):
    query: str
    max_chunks: Optional[int] = 5
    context: Optional[str] = None

class SelectedTextQueryRequest(BaseModel):
    selected_text: str
    query: str

class RAGResponse(BaseModel):
    answer: str
    sources: list
    confidence: Optional[float] = None
    chunks_used: Optional[int] = None
    error: Optional[str] = None

@router.post("/", response_model=RAGResponse)
async def rag_query(request: RAGQueryRequest):
    """
    Process a RAG query with context retrieval from vector database
    """
    try:
        # Check if Qdrant collection exists and has data
        qdrant_client = get_qdrant_client()
        config = get_config()

        try:
            collection_info = qdrant_client.get_collection(config.QDRANT_COLLECTION_NAME)
            if collection_info.points_count == 0:
                # Collection exists but is empty
                logger.warning("Qdrant collection exists but is empty")
                return RAGResponse(
                    answer="No content has been indexed yet. Please run the embedding pipeline first.",
                    sources=[],
                    error="No indexed content available"
                )
        except Exception as e:
            # Collection doesn't exist or other Qdrant error
            logger.error(f"Error checking Qdrant collection: {str(e)}")
            return RAGResponse(
                answer="Embedding not found: Re-run embed pipeline",
                sources=[],
                error="EMBEDDING_NOT_FOUND"
            )

        # Process the query using the RAG agent
        rag_agent = get_rag_agent()
        result = await rag_agent.process_query(
            query=request.query,
            max_chunks=request.max_chunks
        )

        return RAGResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing RAG query: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={
                "error": "Internal server error during RAG query processing",
                "code": "RAG_ERROR",
                "details": str(e)
            }
        )

@router.post("/selected", response_model=RAGResponse)
async def rag_selected_text_query(request: SelectedTextQueryRequest):
    """
    Process a query based only on selected text, bypassing Qdrant
    """
    try:
        # Validate input
        if not request.selected_text or not request.selected_text.strip():
            raise HTTPException(
                status_code=400,
                detail={
                    "error": "Selected text cannot be empty",
                    "code": "INVALID_INPUT",
                    "details": "selected_text parameter is required"
                }
            )

        if not request.query or not request.query.strip():
            raise HTTPException(
                status_code=400,
                detail={
                    "error": "Query cannot be empty",
                    "code": "INVALID_INPUT",
                    "details": "query parameter is required"
                }
            )

        # Process the query using the RAG agent with selected text
        rag_agent = get_rag_agent()
        result = await rag_agent.process_selected_text_query(
            selected_text=request.selected_text,
            query=request.query
        )

        return RAGResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing selected text RAG query: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={
                "error": "Internal server error during selected text RAG query processing",
                "code": "RAG_ERROR",
                "details": str(e)
            }
        )

@router.get("/health")
async def rag_health():
    """
    Health check for the RAG service
    """
    try:
        qdrant_client = get_qdrant_client()
        config = get_config()

        # Try to get collection info to verify Qdrant connection
        collection_info = qdrant_client.get_collection(config.QDRANT_COLLECTION_NAME)

        return {
            "status": "healthy",
            "vector_db": "connected",
            "collection": config.QDRANT_COLLECTION_NAME,
            "points_count": collection_info.points_count
        }
    except Exception as e:
        logger.warning(f"RAG health check failed: {str(e)}")
        return {
            "status": "degraded",
            "vector_db": "disconnected",
            "error": str(e)
        }