from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import os
import sys
import logging
from dotenv import load_dotenv
import uuid
from datetime import datetime
from agents import AsyncOpenAI , OpenAIChatCompletionsModel, RunConfig
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
    description="API for the Book RAG Agent that uses OpenAI Agents SDK with Gemini 2.5 Flash",
    version="1.0.0"
)

# Add CORS middleware to allow requests from the frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "http://localhost:3002",
        "https://physical-ai-and-humanoid-robotics-t-lake.vercel.app",
        ],  # In production, replace with specific frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

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
    source_chunks: List[QueryResult] = []  # Internal only, not displayed in frontend
    session_id: str
    # citations: List[str] = []  # REMOVED: No longer expose citations to prevent metadata leakage

# Define request/response models for authentication
class RegisterRequest(BaseModel):
    email: str
    password: str
    skill_level: Optional[str] = ""
    hardware_background: Optional[str] = ""
    learning_goal: Optional[str] = ""

class LoginRequest(BaseModel):
    email: str
    password: str

class UserResponse(BaseModel):
    id: str
    email: str
    skill_level: Optional[str] = ""
    hardware_background: Optional[str] = ""
    learning_goal: Optional[str] = ""
    created_at: datetime
    updated_at: datetime

class AuthResponse(BaseModel):
    success: bool
    user: Optional[UserResponse] = None
    token: Optional[str] = None
    message: Optional[str] = None
    error: Optional[str] = None
    
    
# Mock storage for users (in a real app, this would be a database)
mock_users_db: Dict[str, Dict[str, Any]] = {}

# connect with gemini 2.5 flash model
gemini_api_key = os.getenv("GEMINI_API_KEY")

#Reference: https://ai.google.dev/gemini-api/docs/openai
if gemini_api_key:
    external_client = AsyncOpenAI( api_key=gemini_api_key,
                                  base_url="https://generativelanguage.googleapis.com/v1beta/openai/", )

    model = OpenAIChatCompletionsModel( model="gemini-2.5-flash", openai_client=external_client )

    config = RunConfig( model=model, model_provider=external_client, tracing_disabled=True )
else:
    external_client = None
    model = None
    config = None
    print("WARNING: GEMINI_API_KEY is not set. The system will run in simulation mode without external API access.")

# Authentication endpoints
@app.post("/api/auth/register", response_model=AuthResponse)
async def register(request: RegisterRequest):
    try:
        # Check if user already exists
        if request.email in mock_users_db:
            raise HTTPException(status_code=400, detail="User already exists")

        # Create a new user (in a real app, this would be stored in a database)
        user_id = str(uuid.uuid4())
        user_data = {
            "id": user_id,
            "email": request.email,
            "password": request.password,  # In a real app, this should be hashed
            "skill_level": request.skill_level,
            "hardware_background": request.hardware_background,
            "learning_goal": request.learning_goal,
            "created_at": datetime.now(),
            "updated_at": datetime.now(),
        }

        mock_users_db[request.email] = user_data

        # Create a mock token (in a real app, this would be a JWT token)
        token = f"mock_token_{user_id}_{int(datetime.now().timestamp())}"

        # Return success response
        user_response = UserResponse(
            id=user_data["id"],
            email=user_data["email"],
            skill_level=user_data["skill_level"],
            hardware_background=user_data["hardware_background"],
            learning_goal=user_data["learning_goal"],
            created_at=user_data["created_at"],
            updated_at=user_data["updated_at"]
        )

        return AuthResponse(
            success=True,
            user=user_response,
            token=token,
            message="User registered successfully"
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Registration error: {str(e)}")
        raise HTTPException(status_code=500, detail="Registration failed")

@app.post("/api/auth/login", response_model=AuthResponse)
async def login(request: LoginRequest):
    try:
        # Check if user exists
        if request.email not in mock_users_db:
            raise HTTPException(status_code=401, detail="Invalid email or password")

        user_data = mock_users_db[request.email]

        # Check password (in a real app, this would involve password hashing)
        if user_data["password"] != request.password:
            raise HTTPException(status_code=401, detail="Invalid email or password")

        # Create a mock token (in a real app, this would be a JWT token)
        token = f"mock_token_{user_data['id']}_{int(datetime.now().timestamp())}"

        # Return success response
        user_response = UserResponse(
            id=user_data["id"],
            email=user_data["email"],
            skill_level=user_data["skill_level"],
            hardware_background=user_data["hardware_background"],
            learning_goal=user_data["learning_goal"],
            created_at=user_data["created_at"],
            updated_at=user_data["updated_at"]
        )

        return AuthResponse(
            success=True,
            user=user_response,
            token=token,
            message="Login successful"
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Login error: {str(e)}")
        raise HTTPException(status_code=500, detail="Login failed")

from fastapi import Request

@app.get("/api/auth/me", response_model=AuthResponse)
async def get_user_info(request: Request):
    try:
        # Extract token from Authorization header
        auth_header = request.headers.get("Authorization")
        if not auth_header or not auth_header.startswith("Bearer "):
            raise HTTPException(status_code=401, detail="Authorization header missing or invalid")

        token = auth_header[7:]  # Remove 'Bearer ' prefix

        # Validate token format (in a real app, this would involve JWT validation)
        if not token.startswith("mock_token_"):
            raise HTTPException(status_code=401, detail="Invalid or expired session token")

        # Find user by token (in a real app, this would be decoded from the JWT)
        user = None
        for email, user_data in mock_users_db.items():
            expected_token = f"mock_token_{user_data['id']}_{int(user_data['updated_at'].timestamp())}" if isinstance(user_data['updated_at'], datetime) else f"mock_token_{user_data['id']}_0"
            if token.startswith(f"mock_token_{user_data['id']}_"):
                user = user_data
                break

        if not user:
            raise HTTPException(status_code=401, detail="Invalid or expired session token")

        # Return user information
        user_response = UserResponse(
            id=user["id"],
            email=user["email"],
            skill_level=user["skill_level"],
            hardware_background=user["hardware_background"],
            learning_goal=user["learning_goal"],
            created_at=user["created_at"],
            updated_at=user["updated_at"]
        )

        return AuthResponse(
            success=True,
            user=user_response
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get user info error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to get user info")

# Dependency to get current user from token
async def get_current_user(request: Request):
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        return None

    token = auth_header[7:]  # Remove 'Bearer ' prefix

    # Validate token format (in a real app, this would involve JWT validation)
    if not token.startswith("mock_token_"):
        return None

    # Find user by token (in a real app, this would be decoded from the JWT)
    for email, user_data in mock_users_db.items():
        if token.startswith(f"mock_token_{user_data['id']}_"):
            return user_data

    return None

# Personalization endpoint
class PersonalizationRequest(BaseModel):
    chapter_content: str
    user_metadata: Dict[str, Any]
    personalization_level: str = "intermediate"
    language: str = "en"

class PersonalizationResponse(BaseModel):
    personalized_content: str

@app.post("/personalize-agent", response_model=PersonalizationResponse)
async def personalize_content(request: PersonalizationRequest, current_user: Dict = Depends(get_current_user)):
    try:
        # In a real implementation, this would call the book_rag_agent with user context
        # to generate personalized content based on user metadata
        # For now, we'll return a mock implementation

        # Extract user metadata
        skill_level = request.user_metadata.get('skill_level', 'beginner')
        hardware_background = request.user_metadata.get('hardware_background', 'general')
        learning_goal = request.user_metadata.get('learning_goal', 'learning')

        # Generate personalized content based on user metadata
        # This would integrate with the existing book_rag_agent in a real implementation
        personalized_content = f"""
# Personalized Content for {current_user['email'] if current_user else 'User'}

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
    return {"message": "Book RAG Agent API is running"}

@app.post("/chat", response_model=AgentResponse)
async def chat(request: AgentRequest):
    # Generate session ID if not provided
    session_id = request.session_id or str(uuid.uuid4())

    # Add user message to session history
    session_manager.add_message_to_session(session_id, {
        "role": "user",
        "content": request.message,
        "selected_text": request.selected_text,
        "timestamp": session_manager.get_current_timestamp()
    })

    # NEW: Check for greeting BEFORE RAG execution
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
            # Execute the selected text search to get results
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
        # Check if API key is available
        current_gemini_api_key = os.getenv("GEMINI_API_KEY")
        if not current_gemini_api_key:
            # If no API key, return a simulated response for testing
            # Use the context from query results to generate a response
            if query_results:
                # If we have source chunks, create a response based on them
                context_snippets = "\n".join([f"- {chunk['content']}" for chunk in query_results[:3]])  # Use top 3 results
                agent_response = f"Based on the provided context:\n{context_snippets}\n\nFor your question '{request.message}', this is what I found in the text."
            else:
                agent_response = f"Simulated response for: {request.message}\n\nThis is a simulated response since no GEMINI_API_KEY is configured. In a real implementation, this would be the response from Gemini 2.5 Flash based on the context provided."
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

                # Extract the response - the exact attribute depends on the agents SDK version
                if hasattr(result, 'final_output'):
                    agent_response = result.final_output
                elif hasattr(result, 'output'):
                    agent_response = result.output
                elif isinstance(result, str):
                    agent_response = result
                else:
                    agent_response = str(result)

                # Strip RAG metadata from response (safety net)
                agent_response = ResponseFormatter.strip_metadata(agent_response)

                # Validate response formatting
                if not ResponseFormatter.validate_no_metadata(agent_response):
                    logger.warning(f"Metadata detected in response after generation (session: {session_id})")

            except Exception as agent_error:
                # If the agent call fails (e.g., API error), return a helpful error message
                logger.error(f"Agent execution failed: {str(agent_error)}")
                agent_response = f"Error: The AI service is currently unavailable. Details: {str(agent_error)}"

        # Format the source chunks for the response
        formatted_chunks = [
            QueryResult(
                id=qr["id"],
                content=qr["content"],
                similarity_score=qr["similarity_score"],
                metadata=qr.get("metadata", {})  # Add any metadata if available
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
        # Handle errors appropriately
        import traceback
        logger.error(f"Error processing request: {str(e)}\n{traceback.format_exc()}")
        return AgentResponse(
            response=f"Error processing request: {str(e)}",
            source_chunks=[],
            session_id=session_id
        )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)