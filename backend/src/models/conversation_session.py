from pydantic import BaseModel
from datetime import datetime
from typing import Optional


class ConversationSessionBase(BaseModel):
    id: str
    user_id: Optional[str] = None
    active: bool = True


class ConversationSessionCreate(ConversationSessionBase):
    pass


class ConversationSessionUpdate(BaseModel):
    active: Optional[bool] = None


class ConversationSession(ConversationSessionBase):
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True