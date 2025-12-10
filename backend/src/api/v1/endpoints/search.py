from fastapi import APIRouter, HTTPException
from slowapi import Limiter
from slowapi.util import get_remote_address
from src.api.v1.schemas.search import SearchRequest, SearchResponse, SearchResult
from src.services.qdrant_service import qdrant_service

# Initialize limiter for this router
limiter = Limiter(key_func=get_remote_address)

router = APIRouter()


@router.post("/", response_model=SearchResponse)
@limiter.limit("20/minute")  # Limit to 20 requests per minute per IP
async def search_chunks(request: SearchRequest):
    """
    Find book content chunks similar to the provided query embedding.
    """
    try:
        # Search for similar chunks in Qdrant
        results = await qdrant_service.search_similar_chunks(
            query_embedding=request.query_embedding,
            top_k=request.top_k
        )

        # Convert to response format
        search_results = []
        for result in results:
            search_result = SearchResult(
                chunk_id=result["id"],
                content=result["content"],
                similarity_score=result["similarity_score"],
                metadata=result.get("metadata", {})
            )
            search_results.append(search_result)

        return SearchResponse(results=search_results, count=len(search_results))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error searching chunks: {str(e)}")