from fastapi import APIRouter, HTTPException
from slowapi import Limiter
from slowapi.util import get_remote_address
from src.api.v1.schemas.embedding import EmbedRequest, EmbedResponse
from src.services.embedding_service import embedding_service

# Initialize limiter for this router
limiter = Limiter(key_func=get_remote_address)

router = APIRouter()


@router.post("/", response_model=EmbedResponse)
@limiter.limit("30/minute")  # Limit to 30 requests per minute per IP
async def create_embedding(request: EmbedRequest):
    """
    Generate embedding for the provided text using the configured embedding model.
    """
    try:
        embedding = await embedding_service.create_embedding(request.text)
        return EmbedResponse(embedding=embedding, model=request.model)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating embedding: {str(e)}")