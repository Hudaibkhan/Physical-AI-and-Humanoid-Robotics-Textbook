from pydantic import BaseModel
from datetime import datetime
from typing import Optional, List, Dict


class TextChunkBase(BaseModel):
    id: str
    book_id: str
    content: str
    chunk_index: int
    embedding: Optional[List[float]] = None
    metadata: Optional[Dict] = {}


class TextChunkCreate(TextChunkBase):
    pass


class TextChunkUpdate(BaseModel):
    content: Optional[str] = None
    chunk_index: Optional[int] = None
    metadata: Optional[Dict] = None


class TextChunk(TextChunkBase):
    created_at: datetime

    class Config:
        from_attributes = True