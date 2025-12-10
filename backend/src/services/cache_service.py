import json
import hashlib
from typing import Any, Optional
from redis import asyncio as aioredis
from src.config import get_config


class CacheService:
    def __init__(self):
        self.redis = None

    async def connect(self):
        """Initialize Redis connection"""
        config = get_config()
        self.redis = aioredis.from_url(
            f"redis://{config.REDIS_HOST}:{config.REDIS_PORT}/{config.REDIS_DB}",
            decode_responses=True
        )

    async def close(self):
        """Close Redis connection"""
        if self.redis:
            await self.redis.close()

    def _generate_cache_key(self, prefix: str, data: str) -> str:
        """Generate a cache key using hash of the data"""
        data_hash = hashlib.sha256(data.encode()).hexdigest()[:16]
        return f"{prefix}:{data_hash}"

    async def get(self, key: str) -> Optional[Any]:
        """Get value from cache"""
        if not self.redis:
            return None

        try:
            cached_data = await self.redis.get(key)
            if cached_data:
                return json.loads(cached_data)
        except Exception:
            # If there's an error with Redis, return None to proceed without cache
            return None

        return None

    async def set(self, key: str, value: Any, ttl: Optional[int] = 3600) -> bool:
        """Set value in cache"""
        if not self.redis:
            return False

        try:
            await self.redis.setex(
                key,
                ttl,
                json.dumps(value, ensure_ascii=False)
            )
            return True
        except Exception:
            # If there's an error with Redis, return False
            return False

    async def delete(self, key: str) -> bool:
        """Delete value from cache"""
        if not self.redis:
            return False

        try:
            result = await self.redis.delete(key)
            return result > 0
        except Exception:
            return False

    async def get_question_response(self, question: str, selected_text: str = "") -> Optional[dict]:
        """Get cached response for a question"""
        # Create a unique key based on question and selected text
        cache_input = f"question:{question}|selected_text:{selected_text}"
        key = self._generate_cache_key("qa", cache_input)
        return await self.get(key)

    async def set_question_response(self, question: str, selected_text: str, response: dict) -> bool:
        """Cache response for a question"""
        # Create a unique key based on question and selected text
        cache_input = f"question:{question}|selected_text:{selected_text}"
        key = self._generate_cache_key("qa", cache_input)
        return await self.set(key, response)


# Global cache service instance
cache_service = CacheService()