from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import os
import sys
import logging
from dotenv import load_dotenv
import uuid
from agents import AsyncOpenAI , OpenAIChatCompletionsModel, RunConfig
from connection import config

# Add the current directory to the path so we can import modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from agent import book_rag_agent, rag_query
from connection import connection_manager, session_manager
from agents import Runner

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
    allow_origins=["*"],  # In production, replace with specific frontend URL
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
    source_chunks: List[QueryResult] = []
    session_id: str
    citations: List[str] = []
    
    
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

        # Execute the agent using the OpenAI Agents SDK
        from agents import Runner
        import os

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
                # Run the agent with the user's message
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

        # Create citations from the source chunks
        citations = [f"Source: {chunk.id}" for chunk in formatted_chunks]

        # Add assistant response to session history
        session_manager.add_message_to_session(session_id, {
            "role": "assistant",
            "content": agent_response,
            "timestamp": session_manager.get_current_timestamp()
        })

        return AgentResponse(
            response=agent_response,
            source_chunks=formatted_chunks,
            session_id=session_id,
            citations=citations
        )
    except Exception as e:
        # Handle errors appropriately
        import traceback
        logger.error(f"Error processing request: {str(e)}\n{traceback.format_exc()}")
        return AgentResponse(
            response=f"Error processing request: {str(e)}",
            source_chunks=[],
            session_id=session_id,
            citations=[]
        )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)