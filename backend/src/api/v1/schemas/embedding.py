from pydantic import BaseModel, Field
from typing import List, Optional


class EmbedRequest(BaseModel):
    text: str = Field(..., min_length=1, max_length=10000, description="Text to generate embedding for")
    model: Optional[str] = Field(default="gemini-embedding-001", description="Embedding model to use")


class EmbedResponse(BaseModel):
    embedding: List[float] = Field(..., min_items=1, description="Generated embedding vector")
    model: str = Field(..., min_length=1, description="Model used to generate the embedding")