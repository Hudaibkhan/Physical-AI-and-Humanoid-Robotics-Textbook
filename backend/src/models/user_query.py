from pydantic import BaseModel
from datetime import datetime
from typing import Optional, List


class UserQueryBase(BaseModel):
    id: str
    session_id: str
    query_text: str
    embedding: Optional[List[float]] = None
    selected_text: Optional[str] = ""


class UserQueryCreate(UserQueryBase):
    pass


class UserQueryUpdate(BaseModel):
    query_text: Optional[str] = None
    selected_text: Optional[str] = None


class UserQuery(UserQueryBase):
    timestamp: datetime

    class Config:
        from_attributes = True