from fastapi import APIRouter, HTTPException
from src.services.session_service import session_service
from pydantic import BaseModel
from typing import List, Optional

router = APIRouter()


class GetHistoryRequest(BaseModel):
    session_id: str
    limit: Optional[int] = 10


class Message(BaseModel):
    id: str
    role: str
    content: str
    timestamp: str
    selected_text: Optional[str] = None


class GetHistoryResponse(BaseModel):
    messages: List[Message]
    session_id: str


@router.post("/history", response_model=GetHistoryResponse)
async def get_conversation_history(request: GetHistoryRequest):
    """
    Get conversation history for a session.
    """
    try:
        messages = await session_service.get_conversation_history(
            session_id=request.session_id,
            limit=request.limit
        )

        # Convert to response format
        formatted_messages = []
        for msg in messages:
            formatted_msg = Message(
                id=msg.get("id", ""),
                role=msg.get("role", ""),
                content=msg.get("content", ""),
                timestamp=msg.get("timestamp", ""),
                selected_text=msg.get("selected_text")
            )
            formatted_messages.append(formatted_msg)

        return GetHistoryResponse(
            messages=formatted_messages,
            session_id=request.session_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving conversation history: {str(e)}")


@router.post("/create", response_model=dict)
async def create_session():
    """
    Create a new session.
    """
    try:
        session_id = session_service.generate_session_id()
        session_data = await session_service.create_session(session_id)

        return {
            "session_id": session_id,
            "created": True
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating session: {str(e)}")


@router.post("/extend", response_model=dict)
async def extend_session(session_id: str):
    """
    Extend session TTL.
    """
    try:
        success = await session_service.extend_session(session_id)

        if not success:
            raise HTTPException(status_code=404, detail="Session not found")

        return {
            "session_id": session_id,
            "extended": True
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error extending session: {str(e)}")