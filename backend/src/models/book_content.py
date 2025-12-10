from pydantic import BaseModel
from datetime import datetime
from typing import Optional


class BookContentBase(BaseModel):
    id: str
    title: str
    content: str
    format: str = "markdown"


class BookContentCreate(BookContentBase):
    pass


class BookContentUpdate(BaseModel):
    title: Optional[str] = None
    content: Optional[str] = None


class BookContent(BookContentBase):
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True