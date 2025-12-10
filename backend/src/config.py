import os
from typing import List, Optional
import logging
from pydantic import BaseModel, validator

logger = logging.getLogger(__name__)

class Config:
    """
    Configuration class for the RAG Chatbot application
    """
    # Required environment variables
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")

    # Optional environment variables with defaults
    DEBUG: bool = os.getenv("DEBUG", "False").lower() in ("true", "1", "yes")
    PORT: int = int(os.getenv("PORT", "8000"))
    HOST: str = os.getenv("HOST", "0.0.0.0")
    CORS_ORIGINS: str = os.getenv("CORS_ORIGINS", "*")

    # Qdrant configuration
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")
    QDRANT_VECTOR_SIZE: int = int(os.getenv("QDRANT_VECTOR_SIZE", "1024"))  # Cohere embedding dimension

    # Cohere configuration
    COHERE_MODEL: str = os.getenv("COHERE_MODEL", "embed-multilingual-v3.0")

    # Gemini configuration
    GEMINI_MODEL: str = os.getenv("GEMINI_MODEL", "gemini-pro")

    # Application settings
    CHUNK_SIZE: int = int(os.getenv("CHUNK_SIZE", "512"))
    MAX_CHUNKS_TO_RETRIEVE: int = int(os.getenv("MAX_CHUNKS_TO_RETRIEVE", "5"))
    EMBEDDING_SIMILARITY_THRESHOLD: float = float(os.getenv("EMBEDDING_SIMILARITY_THRESHOLD", "0.5"))
    EMBEDDING_MODEL: str = os.getenv("EMBEDDING_MODEL", "cohere")

    # Redis configuration
    REDIS_HOST: str = os.getenv("REDIS_HOST", "localhost")
    REDIS_PORT: int = int(os.getenv("REDIS_PORT", "6379"))
    REDIS_DB: int = int(os.getenv("REDIS_DB", "0"))

    @classmethod
    def validate_required_vars(cls) -> List[str]:
        """
        Validate that all required environment variables are set

        Returns:
            List of missing environment variables
        """
        missing_vars = []

        if not cls.GEMINI_API_KEY:
            missing_vars.append("GEMINI_API_KEY")

        if not cls.QDRANT_URL:
            missing_vars.append("QDRANT_URL")

        if not cls.QDRANT_API_KEY:
            missing_vars.append("QDRANT_API_KEY")

        if not cls.COHERE_API_KEY:
            missing_vars.append("COHERE_API_KEY")

        return missing_vars

    @classmethod
    def validate_and_log(cls):
        """
        Validate configuration and log the status
        """
        missing_vars = cls.validate_required_vars()

        if missing_vars:
            logger.error(f"Missing required environment variables: {', '.join(missing_vars)}")
            raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

        logger.info("All required environment variables are set")

        if cls.DEBUG:
            logger.info("Debug mode is enabled")

        logger.info(f"Application will run on {cls.HOST}:{cls.PORT}")
        logger.info(f"Using Qdrant collection: {cls.QDRANT_COLLECTION_NAME}")
        logger.info(f"Using Cohere model: {cls.COHERE_MODEL}")
        logger.info(f"Using Gemini model: {cls.GEMINI_MODEL}")

    @classmethod
    def get_cors_origins(cls) -> List[str]:
        """
        Get CORS origins from environment variable

        Returns:
            List of allowed origins
        """
        if cls.CORS_ORIGINS == "*":
            return ["*"]
        else:
            return [origin.strip() for origin in cls.CORS_ORIGINS.split(",") if origin.strip()]

def get_config() -> Config:
    """
    Get the application configuration instance
    """
    config = Config()
    config.validate_and_log()
    return config