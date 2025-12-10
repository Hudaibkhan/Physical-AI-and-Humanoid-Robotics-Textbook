from typing import List, Dict, Any, Optional
import logging
from pydantic import BaseModel
from .base import BaseTool, ToolResult
from ..utils.gemini_provider import get_gemini_provider
from ..config import get_config

logger = logging.getLogger(__name__)

class AnswerWithContextTool(BaseTool):
    """
    Tool for generating an answer to a question based on provided context chunks
    """
    def __init__(self):
        super().__init__(
            name="answer_with_context",
            description="Generate an answer to the question based on the provided context chunks"
        )
        self.gemini_provider = get_gemini_provider()
        self.config = get_config()

    async def execute(self, question: str, chunks: List[Dict[str, Any]], max_tokens: Optional[int] = None) -> ToolResult:
        """
        Execute the answer_with_context tool

        Args:
            question: The question to answer
            chunks: List of context chunks to use for answering
            max_tokens: Maximum number of tokens for the response (optional)

        Returns:
            ToolResult containing the generated answer
        """
        try:
            # Validate parameters
            if not question or not question.strip():
                return self.format_result(
                    success=False,
                    error="Question parameter is required and cannot be empty"
                )

            if not chunks:
                return self.format_result(
                    success=False,
                    error="Chunks parameter is required and cannot be empty"
                )

            # Build context from chunks
            context_parts = []
            for i, chunk in enumerate(chunks):
                content = chunk.get("content", "")
                source_file = chunk.get("source_file", "unknown")
                chunk_index = chunk.get("chunk_index", i)

                context_parts.append(
                    f"Source: {source_file} (Chunk {chunk_index})\n"
                    f"Content: {content}\n"
                )

            full_context = "\n".join(context_parts)

            # Create a prompt that includes the question and context
            prompt = (
                f"Based on the following context, please answer the question.\n\n"
                f"Context:\n{full_context}\n\n"
                f"Question: {question}\n\n"
                f"Please provide a comprehensive answer based solely on the provided context. "
                f"If the answer cannot be determined from the context, please state that explicitly."
            )

            # Generate response using Gemini
            messages = [
                {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context. Only use information from the context to answer the question. If the answer cannot be found in the context, say so."},
                {"role": "user", "content": prompt}
            ]

            response = self.gemini_provider.chat_completions_create(
                messages=messages,
                max_tokens=max_tokens,
                temperature=0.7
            )

            # Extract the answer from the response
            answer = response["choices"][0]["message"]["content"]

            logger.info(f"Generated answer for question: '{question[:50]}...'")

            return self.format_result(
                success=True,
                data=answer,
                metadata={
                    "question": question,
                    "context_chunks_count": len(chunks),
                    "response_tokens": response.get("usage", {}).get("completion_tokens", 0)
                }
            )

        except Exception as e:
            logger.error(f"Error in answer_with_context tool: {str(e)}")
            return self.format_result(
                success=False,
                error=f"Error generating answer with context: {str(e)}"
            )

    def validate_parameters(self, params: Dict[str, Any]) -> bool:
        """
        Validate the parameters for the answer_with_context tool

        Args:
            params: Dictionary of parameters to validate

        Returns:
            True if parameters are valid, False otherwise
        """
        required_params = ["question", "chunks"]
        for param in required_params:
            if param not in params or params[param] is None:
                self.logger.error(f"Missing required parameter: {param}")
                return False

        # Validate question is not empty
        if not params["question"] or not str(params["question"]).strip():
            self.logger.error("Question parameter cannot be empty")
            return False

        # Validate chunks is a list and not empty
        if not isinstance(params["chunks"], list) or len(params["chunks"]) == 0:
            self.logger.error("Chunks parameter must be a non-empty list")
            return False

        return True

# Register the tool in the global registry
answer_with_context_tool = AnswerWithContextTool()
from .base import get_tool_registry
get_tool_registry().register(answer_with_context_tool)