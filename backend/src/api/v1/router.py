from fastapi import APIRouter
from src.api.v1.endpoints import embed, search, ask_agent, health, sessions, metrics

# Create API router
api_router = APIRouter()

# Include endpoints
api_router.include_router(embed.router, tags=["embed"])
api_router.include_router(search.router, tags=["search"])
api_router.include_router(ask_agent.router, tags=["ask-agent"])
api_router.include_router(health.router, tags=["health"])
api_router.include_router(sessions.router, tags=["sessions"], prefix="/sessions")
api_router.include_router(metrics.router, tags=["metrics"], prefix="/metrics")