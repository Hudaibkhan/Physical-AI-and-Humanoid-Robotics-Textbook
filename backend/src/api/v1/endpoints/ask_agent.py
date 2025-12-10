from fastapi import APIRouter, HTTPException
from slowapi import Limiter
from slowapi.util import get_remote_address
from src.api.v1.schemas.chat import AskAgentRequest, AskAgentResponse, SourceChunk
from src.services.rag_service import rag_service
import uuid

# Initialize limiter for this router
limiter = Limiter(key_func=get_remote_address)

router = APIRouter()


@router.post("/", response_model=AskAgentResponse)
@limiter.limit("10/minute")  # Limit to 10 requests per minute per IP
async def ask_agent(request: AskAgentRequest):
    """
    Process user query with RAG: takes a question, retrieves relevant context,
    and generates an answer using the LLM.
    """
    try:
        # Process the question through RAG pipeline with session management
        result = await rag_service.answer_question(
            question=request.question,
            selected_text=request.selected_text if request.selected_text else None,
            session_id=request.session_id,  # Pass the session ID if provided
            similarity_threshold=0.3  # Minimum similarity threshold for relevant results
        )

        # Convert source chunks to proper format
        source_chunks = []
        for chunk in result.get("source_chunks", []):
            source_chunk = SourceChunk(
                id=chunk["id"],
                content=chunk["content"],
                similarity_score=chunk["similarity_score"]
            )
            source_chunks.append(source_chunk)

        # Use the session ID from the result (either provided or generated)
        session_id = result.get("session_id", request.session_id or str(uuid.uuid4()))

        return AskAgentResponse(
            response=result["response"],
            source_chunks=source_chunks,
            session_id=session_id,
            confidence=result.get("confidence")
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")