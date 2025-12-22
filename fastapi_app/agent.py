from agents import Agent, Runner, function_tool
from agents.extensions.models.litellm_model import LitellmModel
from litellm import completion
from pydantic import BaseModel
from typing import List, Dict
import os
try:
    from .connection import connection_manager, external_client, model, config
except ImportError:
    from connection import connection_manager, external_client, model, config
import logging

logger = logging.getLogger(__name__)

# Define structured output type for the rag_query tool
class RagQueryResult(BaseModel):
    id: str
    text: str
    similarity_score: float

@function_tool
async def rag_query(query: str, mode: str, top_k: int = 5, selected_text: str = None) -> List[RagQueryResult]:
    """
    Function tool to query for relevant chunks based on the mode.

    Args:
        query: The user's query
        mode: "rag" for Qdrant search or "selected" for selected text search
        top_k: Number of top results to return
        selected_text: Text to search in selected mode (only used when mode="selected")

    Returns:
        List of RagQueryResult with id, text, and similarity_score
    """
    try:
        if mode == "rag":
            # Convert user query to embedding
            query_embedding = await connection_manager.embed(query)

            # Retrieve from Qdrant
            results = await connection_manager.qdrant_search(query_embedding, top_k)
        elif mode == "selected":
            if not selected_text:
                logger.warning("Selected text mode called without selected_text")
                return []

            # Retrieve from selected text
            results = await connection_manager.selected_text_search(query, selected_text, top_k)
        else:
            raise ValueError(f"Invalid mode: {mode}. Must be 'rag' or 'selected'")

        # Convert results to the expected format
        formatted_results = [
            RagQueryResult(
                id=result["id"],
                text=result["content"],
                similarity_score=result["similarity_score"]
            )
            for result in results
        ]

        logger.info(f"RAG query returned {len(formatted_results)} results in mode: {mode}")
        return formatted_results
    except Exception as e:
        logger.error(f"Error in rag_query: {e}")
        return []

# Create the book_rag_agent with Gemini 2.5 Flash using the external client
# Check if external API is configured
if model is not None:
    book_rag_agent = Agent(
        name="book_rag_agent",
        instructions="""
You are a helpful robotics instructor specializing in Physical AI, ROS 2, and Humanoid Robotics.

RESPONSE STRUCTURE (MANDATORY):
## {Topic Title}

{Brief 2-3 sentence explanation}

### Key Points
- {Point 1}
- {Point 2}
- {Point 3}

### Why It Matters
- {Practical relevance}
- {Application context}

FORMATTING RULES:
- Use markdown headings (## for topic, ### for sections)
- Keep paragraphs to 3-4 sentences maximum
- Use bullet points for lists
- Write in a friendly, calm, confident tone
- Be precise and professional

CONTENT RULES:
- Base answers ONLY on provided context from RAG retrieval
- Do NOT hallucinate information
- If context insufficient, say so clearly
- Do NOT include chunk IDs, source references, or metadata

PROHIBITED in responses:
- "Source: chunk_*"
- "Based on chunk X"
- "Chunk ID: *"
- "Retrieved from: *"
- "Similarity score: *"

Write responses as if you authored them directly, like a robotics instructor teaching a student.
        """,
        model=model,  # Using the configured model from connection.py
        tools=[rag_query]
    )
else:
    # Fallback configuration when API key is not available
    book_rag_agent = Agent(
        name="book_rag_agent",
        instructions="""
You are a helpful robotics instructor specializing in Physical AI, ROS 2, and Humanoid Robotics.

RESPONSE STRUCTURE (MANDATORY):
## {Topic Title}

{Brief 2-3 sentence explanation}

### Key Points
- {Point 1}
- {Point 2}
- {Point 3}

### Why It Matters
- {Practical relevance}
- {Application context}

FORMATTING RULES:
- Use markdown headings (## for topic, ### for sections)
- Keep paragraphs to 3-4 sentences maximum
- Use bullet points for lists
- Write in a friendly, calm, confident tone
- Be precise and professional

CONTENT RULES:
- Base answers ONLY on provided context from RAG retrieval
- Do NOT hallucinate information
- If context insufficient, say so clearly
- Do NOT include chunk IDs, source references, or metadata

PROHIBITED in responses:
- "Source: chunk_*"
- "Based on chunk X"
- "Chunk ID: *"
- "Retrieved from: *"
- "Similarity score: *"

Write responses as if you authored them directly, like a robotics instructor teaching a student.
        """,
        model="gpt-4o-mini",  # Use a standard OpenAI model as fallback when no API key is available
        tools=[rag_query]
    )