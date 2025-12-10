from fastapi import APIRouter, HTTPException, BackgroundTasks
from typing import Optional
import logging
from pydantic import BaseModel
import os
from ..utils.qdrant_client import get_qdrant_client, init_qdrant_collection
from ..utils.embedding_utils import get_embedding_utils
from ..utils.document_loader import load_documents
from ..utils.text_chunker import chunk_text
from ..config import get_config

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/embed", tags=["embed"])

class EmbedRequest(BaseModel):
    source_path: str
    force_recreate: Optional[bool] = False

class EmbedResponse(BaseModel):
    status: str
    chunks_processed: int
    collection_name: str
    message: Optional[str] = None

@router.post("/", response_model=EmbedResponse)
async def embed_documents(request: EmbedRequest, background_tasks: BackgroundTasks):
    """
    Process markdown files, generate embeddings using Cohere, and store in Qdrant
    """
    try:
        # Validate source path
        if not os.path.exists(request.source_path):
            raise HTTPException(
                status_code=400,
                detail={
                    "error": "Source path does not exist",
                    "code": "INVALID_PATH",
                    "details": f"Path {request.source_path} does not exist"
                }
            )

        # Initialize Qdrant collection
        config = get_config()
        init_qdrant_collection(config.QDRANT_COLLECTION_NAME)

        # Load documents
        documents = load_documents(request.source_path)

        # Process documents and create embeddings
        qdrant_client = get_qdrant_client()
        embedding_utils = get_embedding_utils()

        processed_count = 0

        for doc_path, content in documents.items():
            # Chunk the document content
            chunks = chunk_text(content, chunk_size=config.CHUNK_SIZE)

            # Generate embeddings for chunks
            chunk_texts = [chunk["text"] for chunk in chunks]
            embeddings = embedding_utils.embed_text(chunk_texts)

            # Prepare points for Qdrant
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                point = {
                    "id": f"{doc_path}_chunk_{i}",
                    "vector": embedding,
                    "payload": {
                        "content": chunk["text"],
                        "source_file": doc_path,
                        "chunk_index": i,
                        "metadata": {
                            "start_pos": chunk["start_pos"],
                            "end_pos": chunk["end_pos"]
                        }
                    }
                }
                points.append(point)

            # Upsert points to Qdrant
            qdrant_client.upsert(
                collection_name=config.QDRANT_COLLECTION_NAME,
                points=points
            )

            processed_count += len(chunks)

        logger.info(f"Processed {len(documents)} documents, {processed_count} chunks")

        return EmbedResponse(
            status="success",
            chunks_processed=processed_count,
            collection_name=config.QDRANT_COLLECTION_NAME,
            message=f"Successfully processed {len(documents)} documents and {processed_count} chunks"
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in embed endpoint: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={
                "error": "Internal server error during embedding process",
                "code": "EMBEDDING_ERROR",
                "details": str(e)
            }
        )

@router.get("/verify")
async def verify_embeddings():
    """
    Verify if embeddings exist in the vector database
    """
    try:
        qdrant_client = get_qdrant_client()
        config = get_config()

        try:
            collection_info = qdrant_client.get_collection(config.QDRANT_COLLECTION_NAME)
            has_embeddings = collection_info.points_count > 0

            return {
                "has_embeddings": has_embeddings,
                "collection": config.QDRANT_COLLECTION_NAME,
                "points_count": collection_info.points_count
            }
        except Exception:
            # Collection doesn't exist
            return {
                "has_embeddings": False,
                "collection": config.QDRANT_COLLECTION_NAME,
                "points_count": 0,
                "message": "Embedding not found: Re-run embed pipeline"
            }

    except Exception as e:
        logger.error(f"Error verifying embeddings: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={
                "error": "Internal server error during embedding verification",
                "code": "VERIFICATION_ERROR",
                "details": str(e)
            }
        )