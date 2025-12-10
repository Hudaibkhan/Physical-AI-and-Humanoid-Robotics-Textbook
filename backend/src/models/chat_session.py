from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid

class ChatMessage(BaseModel):
    """
    Represents a single message in a chat session
    """
    id: str = str(uuid.uuid4())
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime = datetime.utcnow()
    metadata: Optional[Dict[str, Any]] = None

class ChatSession(BaseModel):
    """
    Represents a chat session between a user and the AI agent
    """
    id: str = str(uuid.uuid4())
    user_id: Optional[str] = None
    messages: List[ChatMessage] = []
    created_at: datetime = datetime.utcnow()
    updated_at: datetime = datetime.utcnow()
    is_active: bool = True
    metadata: Optional[Dict[str, Any]] = None

    def add_message(self, role: str, content: str, metadata: Optional[Dict[str, Any]] = None):
        """
        Add a message to the session

        Args:
            role: The role of the message sender ("user" or "assistant")
            content: The message content
            metadata: Optional metadata for the message
        """
        message = ChatMessage(
            role=role,
            content=content,
            timestamp=datetime.utcnow(),
            metadata=metadata
        )
        self.messages.append(message)
        self.updated_at = datetime.utcnow()

    def get_context(self, max_messages: int = 10) -> List[Dict[str, str]]:
        """
        Get the recent context from the session

        Args:
            max_messages: Maximum number of recent messages to return

        Returns:
            List of message dictionaries with role and content
        """
        recent_messages = self.messages[-max_messages:] if len(self.messages) > max_messages else self.messages
        return [{"role": msg.role, "content": msg.content} for msg in recent_messages]

    def get_last_user_message(self) -> Optional[ChatMessage]:
        """
        Get the last user message in the session

        Returns:
            The last user message or None if no user messages exist
        """
        for message in reversed(self.messages):
            if message.role == "user":
                return message
        return None

    def get_last_assistant_message(self) -> Optional[ChatMessage]:
        """
        Get the last assistant message in the session

        Returns:
            The last assistant message or None if no assistant messages exist
        """
        for message in reversed(self.messages):
            if message.role == "assistant":
                return message
        return None

class ChatSessionManager:
    """
    Manages chat sessions in memory (in a real application, this would use a database)
    """
    def __init__(self):
        self.sessions: Dict[str, ChatSession] = {}

    def create_session(self, user_id: Optional[str] = None, metadata: Optional[Dict[str, Any]] = None) -> ChatSession:
        """
        Create a new chat session

        Args:
            user_id: Optional user identifier
            metadata: Optional session metadata

        Returns:
            The created chat session
        """
        session = ChatSession(
            user_id=user_id,
            metadata=metadata
        )
        self.sessions[session.id] = session
        return session

    def get_session(self, session_id: str) -> Optional[ChatSession]:
        """
        Get a chat session by ID

        Args:
            session_id: The session identifier

        Returns:
            The chat session or None if not found
        """
        return self.sessions.get(session_id)

    def update_session(self, session: ChatSession):
        """
        Update a chat session

        Args:
            session: The updated session object
        """
        self.sessions[session.id] = session

    def delete_session(self, session_id: str):
        """
        Delete a chat session

        Args:
            session_id: The session identifier to delete
        """
        if session_id in self.sessions:
            del self.sessions[session_id]

    def add_message_to_session(self, session_id: str, role: str, content: str, metadata: Optional[Dict[str, Any]] = None):
        """
        Add a message to a specific session

        Args:
            session_id: The session identifier
            role: The role of the message sender
            content: The message content
            metadata: Optional message metadata
        """
        session = self.get_session(session_id)
        if session:
            session.add_message(role, content, metadata)
            self.update_session(session)

# Global instance for reuse
chat_session_manager = ChatSessionManager()

def get_chat_session_manager() -> ChatSessionManager:
    """
    Get the chat session manager instance
    """
    return chat_session_manager