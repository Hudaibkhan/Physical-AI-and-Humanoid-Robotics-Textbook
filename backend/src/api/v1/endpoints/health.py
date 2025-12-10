from fastapi import APIRouter
from datetime import datetime

router = APIRouter()


@router.get("/")
async def health_check():
    """
    Returns the health status of the API.
    """
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat()
    }