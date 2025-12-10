from typing import List, Dict, Any, Optional
import logging
from pydantic import BaseModel
from .base import BaseTool, ToolResult
from ..utils.qdrant_client import get_qdrant_client
from ..utils.embedding_utils import get_embedding_utils
from ..config import get_config

logger = logging.getLogger(__name__)

class RetrieveChunksTool(BaseTool):
    """
    Tool for retrieving relevant content chunks from the vector database based on a query
    """
    def __init__(self):
        super().__init__(
            name="retrieve_chunks",
            description="Retrieve relevant text chunks from the vector database based on the query"
        )
        self.qdrant_client = get_qdrant_client()
        self.embedding_utils = get_embedding_utils()
        self.config = get_config()

    async def execute(self, query: str, limit: Optional[int] = None, threshold: Optional[float] = None) -> ToolResult:
        """
        Execute the retrieve_chunks tool

        Args:
            query: The search query to find relevant chunks
            limit: Maximum number of chunks to retrieve (defaults to config value)
            threshold: Minimum similarity threshold for retrieved chunks (defaults to config value)

        Returns:
            ToolResult containing a list of relevant text chunks
        """
        try:
            # Validate parameters
            if not query or not query.strip():
                return self.format_result(
                    success=False,
                    error="Query parameter is required and cannot be empty"
                )

            # Use default values if not provided
            if limit is None:
                limit = self.config.MAX_CHUNKS_TO_RETRIEVE
            if threshold is None:
                # Use a lower default threshold since semantic search scores can be low
                threshold = 0.1  # Instead of using config's default of 0.5

            # Generate embedding for the query
            query_embedding = self.embedding_utils.embed_single_text(query)

            # Query in Qdrant for similar chunks using vector search
            search_results = self.qdrant_client.query_points(
                collection_name=self.config.QDRANT_COLLECTION_NAME,
                query=query_embedding,
                limit=limit,
                score_threshold=threshold
            )

            # Extract content from search results
            chunks = []
            for result in search_results.points:
                # Use the threshold for filtering (already applied in query, but double-check)
                if result.score >= threshold:
                    chunk_data = {
                        "id": str(result.id),
                        "content": result.payload.get("content", "") if result.payload else "",
                        "source_file": result.payload.get("source_file", "") if result.payload else "",
                        "chunk_index": result.payload.get("chunk_index", 0) if result.payload else 0,
                        "score": result.score,
                        "metadata": result.payload.get("metadata", {}) if result.payload else {}
                    }
                    chunks.append(chunk_data)

            logger.info(f"Retrieved {len(chunks)} chunks for query: '{query[:50]}...'")

            return self.format_result(
                success=True,
                data=chunks,
                metadata={
                    "query": query,
                    "retrieved_count": len(chunks),
                    "search_limit": limit,
                    "similarity_threshold": threshold
                }
            )

        except Exception as e:
            logger.error(f"Error in retrieve_chunks tool: {str(e)}")
            return self.format_result(
                success=False,
                error=f"Error retrieving chunks: {str(e)}"
            )

    def validate_parameters(self, params: Dict[str, Any]) -> bool:
        """
        Validate the parameters for the retrieve_chunks tool

        Args:
            params: Dictionary of parameters to validate

        Returns:
            True if parameters are valid, False otherwise
        """
        required_params = ["query"]
        for param in required_params:
            if param not in params or params[param] is None:
                self.logger.error(f"Missing required parameter: {param}")
                return False

        # Validate query is not empty
        if not params["query"] or not str(params["query"]).strip():
            self.logger.error("Query parameter cannot be empty")
            return False

        return True

# Register the tool in the global registry
retrieve_chunks_tool = RetrieveChunksTool()
from .base import get_tool_registry
get_tool_registry().register(retrieve_chunks_tool)