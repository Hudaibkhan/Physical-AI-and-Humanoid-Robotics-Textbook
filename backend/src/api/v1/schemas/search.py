from pydantic import BaseModel, Field
from typing import List, Optional


class SearchRequest(BaseModel):
    query_embedding: List[float] = Field(..., min_items=1, max_items=10000, description="Query embedding vector")
    top_k: Optional[int] = Field(default=5, ge=1, le=100, description="Number of top results to return")
    book_id: Optional[str] = Field(default=None, min_length=1, max_length=100, description="Book ID to search within")


class SearchResult(BaseModel):
    chunk_id: str = Field(..., min_length=1, description="Unique identifier for the chunk")
    content: str = Field(..., min_length=1, max_length=10000, description="Content of the retrieved chunk")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score between 0 and 1")
    metadata: dict = Field(..., description="Additional metadata for the chunk")


class SearchResponse(BaseModel):
    results: List[SearchResult] = Field(..., max_items=100, description="List of search results")
    count: int = Field(..., ge=0, description="Total number of results returned")