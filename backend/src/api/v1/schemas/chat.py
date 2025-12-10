from pydantic import BaseModel, Field
from typing import List, Optional


class AskAgentRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=5000, description="User's question to the chatbot")
    selected_text: Optional[str] = Field(default="", max_length=10000, description="Text selected by user on the page")
    session_id: Optional[str] = Field(default=None, min_length=1, max_length=100, description="Session identifier for conversation continuity")


class SourceChunk(BaseModel):
    id: str = Field(..., min_length=1, description="Unique identifier for the source chunk")
    content: str = Field(..., min_length=1, max_length=10000, description="Content of the source chunk")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score between 0 and 1")


class AskAgentResponse(BaseModel):
    response: str = Field(..., min_length=1, max_length=10000, description="Response from the chatbot")
    source_chunks: Optional[List[SourceChunk]] = Field(default=[], max_items=20, description="Source chunks used to generate the response")
    session_id: str = Field(..., min_length=1, description="Session identifier for conversation continuity")
    confidence: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="Confidence score of the response")


class QueryResponse(BaseModel):
    response: str = Field(..., min_length=1, max_length=10000, description="Response from the query")
    source_chunks: Optional[List[dict]] = Field(default=[], max_items=20, description="Source chunks used to generate the response")
    session_id: str = Field(..., min_length=1, description="Session identifier for conversation continuity")
    confidence: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="Confidence score of the response")