from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import os
import sys
import logging
from dotenv import load_dotenv
import uuid
from datetime import datetime
from agents import AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig
from connection import config, groq_config

# Add the current directory to the path so we can import modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from agent import book_rag_agent, rag_query
from connection import connection_manager, session_manager
from agents import Runner
from llm_router import LLMRouter
from formatters import ResponseFormatter
from greeting_detector import GreetingDetector

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

app = FastAPI(
    title="Book RAG Agent API",
    description="Pure RAG/AI service for the Physical AI and Humanoid Robotics textbook. Authentication is handled by separate Better Auth backend.",
    version="2.0.0"
)

# Get CORS origins from environment variable
# Format: comma-separated list of allowed origins
ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000,http://localhost:3001,http://localhost:3002")
allowed_origins_list = [origin.strip() for origin in ALLOWED_ORIGINS.split(",")]

# Add CORS middleware with environment-based configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response Models for RAG functionality only
class AgentRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None

class QueryResult(BaseModel):
    id: str
    content: str
    similarity_score: float
    metadata: dict = {}

class AgentResponse(BaseModel):
    response: str
    source_chunks: List[QueryResult] = []
    session_id: str

class PersonalizationRequest(BaseModel):
    chapter_content: str
    user_metadata: Dict[str, Any]
    personalization_level: str = "intermediate"
    language: str = "en"

class PersonalizationResponse(BaseModel):
    personalized_content: str

# Connect with Gemini 2.5 Flash model using environment variables
gemini_api_key = os.getenv("GEMINI_API_KEY")
llm_base_url = os.getenv("LLM_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/")

# Reference: https://ai.google.dev/gemini-api/docs/openai
if gemini_api_key:
    external_client = AsyncOpenAI(
        api_key=gemini_api_key,
        base_url=llm_base_url,
    )
    model = OpenAIChatCompletionsModel(
        model="gemini-2.5-flash",
        openai_client=external_client
    )
    config = RunConfig(
        model=model,
        model_provider=external_client,
        tracing_disabled=True
    )
else:
    external_client = None
    model = None
    config = None
    logger.warning("GEMINI_API_KEY is not set. The system will run in simulation mode without external API access.")

# Personalization endpoint
@app.post("/personalize-agent", response_model=PersonalizationResponse)
async def personalize_content(request: PersonalizationRequest):
    """
    Personalizes content based on user metadata.
    Authentication is handled by the frontend - this endpoint trusts the user_metadata provided.
    No authentication logic - expects frontend to provide verified user_metadata.
    """
    try:
        # Extract user metadata (provided by authenticated frontend)
        skill_level = request.user_metadata.get('skill_level', 'beginner')
        hardware_background = request.user_metadata.get('hardware_background', 'general')
        learning_goal = request.user_metadata.get('learning_goal', 'learning')

        # Generate personalized content based on user metadata
        personalized_content = f"""
# Personalized Content

Based on your profile:
- **Skill Level**: {skill_level}
- **Hardware Background**: {hardware_background}
- **Learning Goal**: {learning_goal}

## Personalized Chapter Content

{request.chapter_content}

### Adapted for Your Background

The content above has been personalized based on your experience level and interests. Examples and explanations have been adjusted to match your background in {hardware_background} and your learning goal of {learning_goal}. The complexity has been adapted to your skill level ({skill_level}).
        """.strip()

        return PersonalizationResponse(personalized_content=personalized_content)
    except Exception as e:
        logger.error(f"Personalization error: {str(e)}")
        raise HTTPException(status_code=500, detail="Personalization failed")

@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "message": "Book RAG Agent API is running",
        "version": "2.0.0",
        "service": "RAG/AI only - Authentication handled separately",
        "endpoints": {
            "chat": "/chat",
            "personalize": "/personalize-agent",
            "health": "/health",
            "docs": "/docs"
        }
    }

@app.get("/health")
async def health_check():
    """Detailed health check with service status"""
    return {
        "status": "healthy",
        "services": {
            "gemini": gemini_api_key is not None,
            "qdrant": os.getenv("QDRANT_HOST") is not None,
            "groq_fallback": os.getenv("GROQ_API_KEY") is not None
        },
        "config": {
            "llm_base_url": llm_base_url,
            "allowed_origins": len(allowed_origins_list),
            "port": os.getenv("PORT", "8000"),
            "environment": os.getenv("ENVIRONMENT", "development")
        },
        "timestamp": datetime.now().isoformat()
    }

@app.post("/chat", response_model=AgentResponse)
async def chat(request: AgentRequest):
    """
    Main RAG chat endpoint.
    No authentication required - trusts frontend to only send authorized requests.
    Focuses purely on RAG functionality: retrieval + generation.
    """
    # Generate session ID if not provided
    session_id = request.session_id or str(uuid.uuid4())

    # Add user message to session history
    session_manager.add_message_to_session(session_id, {
        "role": "user",
        "content": request.message,
        "selected_text": request.selected_text,
        "timestamp": session_manager.get_current_timestamp()
    })

    # Check for greeting BEFORE RAG execution
    if GreetingDetector.is_greeting(request.message):
        logger.info(f"Greeting detected: '{request.message[:50]}' - skipping RAG")
        greeting_response = GreetingDetector.generate_greeting_response()

        # Add assistant response to session history
        session_manager.add_message_to_session(session_id, {
            "role": "assistant",
            "content": greeting_response,
            "timestamp": session_manager.get_current_timestamp()
        })

        return AgentResponse(
            response=greeting_response,
            source_chunks=[],
            session_id=session_id
        )

    try:
        # Determine mode based on presence of selected_text and prepare context
        query_results = []
        if request.selected_text:
            # Selected text mode - use the provided text directly
            query_results = await connection_manager.direct_rag_search(
                query=request.message,
                mode="selected",
                top_k=5,
                selected_text=request.selected_text
            )
        else:
            # Normal RAG mode - search in Qdrant
            query_results = await connection_manager.direct_rag_search(
                query=request.message,
                mode="rag",
                top_k=5
            )

        # Execute the agent using the OpenAI Agents SDK with LLM fallback
        current_gemini_api_key = os.getenv("GEMINI_API_KEY")
        if not current_gemini_api_key:
            # If no API key, return a simulated response for testing
            if query_results:
                context_snippets = "\n".join([f"- {chunk['content']}" for chunk in query_results[:3]])
                agent_response = f"Based on the provided context:\n{context_snippets}\n\nFor your question '{request.message}', this is what I found in the text."
            else:
                agent_response = f"Simulated response for: {request.message}\n\nThis is a simulated response since no GEMINI_API_KEY is configured."
        else:
            try:
                # Use LLMRouter for automatic Gemini → Groq fallback
                if groq_config:
                    llm_router = LLMRouter(config, groq_config)
                    result = await llm_router.run_with_fallback(book_rag_agent, request.message)
                else:
                    # No fallback available, use Gemini only
                    result = await Runner.run(
                        book_rag_agent,
                        input=request.message,
                        run_config=config
                    )

                # Extract the response
                if hasattr(result, 'final_output'):
                    agent_response = result.final_output
                elif hasattr(result, 'output'):
                    agent_response = result.output
                elif isinstance(result, str):
                    agent_response = result
                else:
                    agent_response = str(result)

                # Strip RAG metadata from response
                agent_response = ResponseFormatter.strip_metadata(agent_response)

                # Validate response formatting
                if not ResponseFormatter.validate_no_metadata(agent_response):
                    logger.warning(f"Metadata detected in response after generation (session: {session_id})")

            except Exception as agent_error:
                logger.error(f"Agent execution failed: {str(agent_error)}")
                agent_response = f"Error: The AI service is currently unavailable. Details: {str(agent_error)}"

        # Format the source chunks for the response
        formatted_chunks = [
            QueryResult(
                id=qr["id"],
                content=qr["content"],
                similarity_score=qr["similarity_score"],
                metadata=qr.get("metadata", {})
            )
            for qr in query_results
        ]

        # Add assistant response to session history
        session_manager.add_message_to_session(session_id, {
            "role": "assistant",
            "content": agent_response,
            "timestamp": session_manager.get_current_timestamp()
        })

        return AgentResponse(
            response=agent_response,
            source_chunks=formatted_chunks,
            session_id=session_id
        )
    except Exception as e:
        import traceback
        logger.error(f"Error processing request: {str(e)}\n{traceback.format_exc()}")
        return AgentResponse(
            response=f"Error processing request: {str(e)}",
            source_chunks=[],
            session_id=session_id
        )

if __name__ == "__main__":
    import uvicorn
    # Use environment variables for host and port configuration
    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")
    reload = os.getenv("RELOAD", "false").lower() == "true"

    logger.info(f"Starting FastAPI server on {host}:{port}")
    logger.info(f"Environment: {os.getenv('ENVIRONMENT', 'development')}")
    logger.info(f"Allowed CORS origins: {allowed_origins_list}")

    uvicorn.run(app, host=host, port=port, reload=reload)
