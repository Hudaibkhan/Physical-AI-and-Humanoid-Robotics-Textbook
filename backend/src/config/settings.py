
from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # Qdrant settings
    qdrant_url: str = "https://88b96840-d5e8-477c-9fd2-4fbd361591ee.us-east4-0.gcp.cloud.qdrant.io"
    qdrant_api_key: Optional[str] = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.5ezXFLCiBuQxCu7f50ZHOMmMO6HbtX7dTG75KTuVrSY"
    collection_name: str = "book_chunks"

    # Gemini settings
    gemini_api_key: str = "AIzaSyDCiLd0McLcljmMcOegzuZznxFOD_WuRhc"
    gemini_model: str = "gemini-2.5-flash"

    # Embedding model settings
    cohere_api_key: str = "bQQqcITgyutTE6IgsxlLUsGdudFxy9TwLpwfbuzP"
    embedding_model: str = "cohere"  # Use "cohere" to indicate using Cohere embeddings

    # Redis settings for caching
    redis_host: str = "localhost"
    redis_port: int = 6379
    redis_db: int = 0
    redis_password: Optional[str] = None
    cache_ttl: int = 3600  # Cache time-to-live in seconds (1 hour)

    # Application settings
    app_name: str = "RAG Chatbot API"
    debug: bool = False
    max_concurrent_users: int = 50

    class Config:
        env_file = ".env"


settings = Settings()