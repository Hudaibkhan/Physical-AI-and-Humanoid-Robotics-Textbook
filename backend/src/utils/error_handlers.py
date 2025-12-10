from fastapi import HTTPException, Request
from fastapi.responses import JSONResponse
from typing import Dict, Any
import logging
import traceback
from pydantic import BaseModel

logger = logging.getLogger(__name__)

class ErrorResponse(BaseModel):
    """
    Standard error response model
    """
    error: str
    code: str
    details: Dict[str, Any] = {}

def handle_error(error_code: str, error_message: str, details: Dict[str, Any] = None) -> Dict[str, Any]:
    """
    Standard error handler that returns a consistent error format

    Args:
        error_code: Standardized error code
        error_message: Human-readable error message
        details: Additional error details

    Returns:
        Dictionary with error information
    """
    if details is None:
        details = {}

    error_response = {
        "error": error_message,
        "code": error_code,
        "details": details
    }

    logger.error(f"{error_code}: {error_message} - Details: {details}")
    return error_response

def handle_exception(exc: Exception, context: str = "") -> Dict[str, Any]:
    """
    Handle exceptions and return standardized error response

    Args:
        exc: The exception that occurred
        context: Context where the error occurred

    Returns:
        Dictionary with error information
    """
    error_msg = str(exc)
    error_type = type(exc).__name__

    logger.error(f"Exception in {context}: {error_type} - {error_msg}")
    logger.error(f"Traceback: {traceback.format_exc()}")

    # Map specific exceptions to error codes
    if error_type == "AuthenticationError":
        error_code = "AUTH_ERROR"
    elif error_type == "ValueError":
        error_code = "INVALID_INPUT"
    elif error_type == "ConnectionError":
        error_code = "CONNECTION_ERROR"
    elif error_type == "TimeoutError":
        error_code = "TIMEOUT_ERROR"
    elif "Qdrant" in error_type or "qdrant" in error_msg.lower():
        error_code = "VECTOR_DB_ERROR"
    elif "Cohere" in error_type or "cohere" in error_msg.lower():
        error_code = "EMBEDDING_ERROR"
    elif "Gemini" in error_type or "gemini" in error_msg.lower() or "Google" in error_type:
        error_code = "LLM_ERROR"
    else:
        error_code = "INTERNAL_ERROR"

    return handle_error(
        error_code=error_code,
        error_message=error_msg,
        details={
            "exception_type": error_type,
            "context": context,
            "traceback": traceback.format_exc() if error_code == "INTERNAL_ERROR" else None
        }
    )

def create_http_exception(status_code: int, error_code: str, error_message: str, details: Dict[str, Any] = None) -> HTTPException:
    """
    Create an HTTPException with standardized error format

    Args:
        status_code: HTTP status code
        error_code: Standardized error code
        error_message: Human-readable error message
        details: Additional error details

    Returns:
        HTTPException instance
    """
    if details is None:
        details = {}

    return HTTPException(
        status_code=status_code,
        detail={
            "error": error_message,
            "code": error_code,
            "details": details
        }
    )

def embedding_not_found_error() -> Dict[str, Any]:
    """
    Return standardized error for when embeddings are not found

    Returns:
        Dictionary with embedding not found error
    """
    return handle_error(
        error_code="EMBEDDING_NOT_FOUND",
        error_message="Embedding not found: Re-run embed pipeline",
        details={
            "suggestion": "Run the embedding pipeline to index your content"
        }
    )

def qdrant_connection_error() -> Dict[str, Any]:
    """
    Return standardized error for Qdrant connection issues

    Returns:
        Dictionary with Qdrant connection error
    """
    return handle_error(
        error_code="QDRANT_CONNECTION_ERROR",
        error_message="Unable to connect to vector database",
        details={
            "suggestion": "Check QDRANT_URL and QDRANT_API_KEY in environment variables"
        }
    )

def llm_provider_error() -> Dict[str, Any]:
    """
    Return standardized error for LLM provider issues

    Returns:
        Dictionary with LLM provider error
    """
    return handle_error(
        error_code="LLM_PROVIDER_ERROR",
        error_message="Unable to connect to language model provider",
        details={
            "suggestion": "Check GEMINI_API_KEY in environment variables"
        }
    )

def cohere_api_error() -> Dict[str, Any]:
    """
    Return standardized error for Cohere API issues

    Returns:
        Dictionary with Cohere API error
    """
    return handle_error(
        error_code="COHERE_API_ERROR",
        error_message="Unable to connect to embedding service",
        details={
            "suggestion": "Check COHERE_API_KEY in environment variables"
        }
    )