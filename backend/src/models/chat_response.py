from pydantic import BaseModel
from datetime import datetime
from typing import Optional, List


class ChatResponseBase(BaseModel):
    id: str
    query_id: str
    response_text: str
    source_chunks: Optional[List[str]] = []
    confidence: Optional[float] = None


class ChatResponseCreate(ChatResponseBase):
    pass


class ChatResponseUpdate(BaseModel):
    response_text: Optional[str] = None
    confidence: Optional[float] = None


class ChatResponse(ChatResponseBase):
    timestamp: datetime

    class Config:
        from_attributes = True