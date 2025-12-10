import json
import uuid
from datetime import datetime, timedelta
from typing import Dict, List, Optional
from src.config import get_config
from src.services.cache_service import cache_service


class SessionService:
    def __init__(self):
        self.cache = cache_service
        # Use a default TTL of 1 hour (3600 seconds) if not specified
        self.session_ttl = 3600  # Use a default TTL of 1 hour (3600 seconds)

    def generate_session_id(self) -> str:
        """Generate a unique session ID"""
        return str(uuid.uuid4())

    async def create_session(self, session_id: str, user_id: Optional[str] = None) -> Dict:
        """Create a new session"""
        session_data = {
            "session_id": session_id,
            "user_id": user_id,
            "created_at": datetime.utcnow().isoformat(),
            "last_activity": datetime.utcnow().isoformat(),
            "conversation_history": [],
            "metadata": {}
        }

        # Store session in cache
        await self.cache.set(f"session:{session_id}", session_data, self.session_ttl)
        return session_data

    async def get_session(self, session_id: str) -> Optional[Dict]:
        """Retrieve session data"""
        session_data = await self.cache.get(f"session:{session_id}")
        return session_data

    async def update_session(self, session_id: str, updates: Dict) -> bool:
        """Update session data"""
        session_data = await self.get_session(session_id)
        if not session_data:
            return False

        # Update with provided data
        session_data.update(updates)
        session_data["last_activity"] = datetime.utcnow().isoformat()

        # Store updated session
        await self.cache.set(f"session:{session_id}", session_data, self.session_ttl)
        return True

    async def add_message_to_session(self, session_id: str, message: Dict) -> bool:
        """Add a message to the conversation history"""
        session_data = await self.get_session(session_id)
        if not session_data:
            return False

        # Add message to history
        session_data["conversation_history"].append({
            "id": str(uuid.uuid4()),
            "timestamp": datetime.utcnow().isoformat(),
            **message
        })

        # Limit history to last 50 messages to prevent memory issues
        session_data["conversation_history"] = session_data["conversation_history"][-50:]

        # Update session
        await self.cache.set(f"session:{session_id}", session_data, self.session_ttl)
        return True

    async def get_conversation_history(self, session_id: str, limit: int = 10) -> List[Dict]:
        """Get conversation history for a session"""
        session_data = await self.get_session(session_id)
        if not session_data:
            return []

        history = session_data.get("conversation_history", [])
        # Return last 'limit' messages
        return history[-limit:] if len(history) > limit else history

    async def clear_session(self, session_id: str) -> bool:
        """Clear session data"""
        return await self.cache.delete(f"session:{session_id}")

    async def extend_session(self, session_id: str) -> bool:
        """Extend session TTL"""
        session_data = await self.get_session(session_id)
        if not session_data:
            return False

        # Update last activity and extend TTL
        session_data["last_activity"] = datetime.utcnow().isoformat()
        await self.cache.set(f"session:{session_id}", session_data, self.session_ttl)
        return True


# Global instance
session_service = SessionService()