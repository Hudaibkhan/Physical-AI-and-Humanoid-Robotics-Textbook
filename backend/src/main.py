from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager
import os
import logging
import time
from dotenv import load_dotenv

# Load environment variables first
load_dotenv()

from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import and get config instance after loading environment variables
from src.config import get_config
config = get_config()

from src.services.cache_service import cache_service
from src.services.metrics_service import metrics_service

from src.api.v1.router import api_router

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup logic
    print("Starting up RAG Chatbot API...")

    # Initialize services here if needed
    # Example: await initialize_qdrant_client()
    await cache_service.connect()

    yield

    # Shutdown logic
    print("Shutting down RAG Chatbot API...")
    await cache_service.close()

# Create FastAPI app instance with security considerations
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Retrieval-Augmented Generation chatbot system",
    version="1.0.0",
    lifespan=lifespan,
    # Add security-related configurations
    root_path=""  # Set if running behind a proxy
)

# Add security headers middleware
@app.middleware("http")
async def add_security_headers(request, call_next):
    response = await call_next(request)

    # Add security headers
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-XSS-Protection"] = "1; mode=block"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["X-Permitted-Cross-Domain-Policies"] = "none"
    response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
    response.headers["Permissions-Policy"] = "geolocation=(), microphone=(), camera=()"

    # Add HSTS header (only for HTTPS in production)
    # In production, make sure to set the correct max-age and include subdomains if needed
    if request.url.scheme == "https":
        hsts_header = f"max-age=31536000; includeSubDomains"  # 1 year
        response.headers["Strict-Transport-Security"] = hsts_header

    return response

# Add TrustedHost middleware for production
if not config.DEBUG:
    app.add_middleware(
        TrustedHostMiddleware,
        allowed_hosts=["localhost", "127.0.0.1", ".render.com", os.getenv("ALLOWED_HOST", "")],
    )

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=config.get_cors_origins(),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API router
app.include_router(api_router, prefix="/api/v1")

# Include RAG and embed routes
from .api import rag_routes, embed_routes
app.include_router(rag_routes.router, prefix="/api")
app.include_router(embed_routes.router, prefix="/api")

# Add rate limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Add performance monitoring and metrics collection middleware
@app.middleware("http")
async def add_process_time_header(request, call_next):
    start_time = time.time()
    metrics_service.record_request(request.url.path, request.method)

    try:
        response = await call_next(request)
        process_time = time.time() - start_time

        # Record response time
        metrics_service.record_response_time(request.url.path, request.method, process_time)

        # Add response time header
        response.headers["X-Process-Time"] = f"{process_time:.3f}"

        # Log slow requests for monitoring
        if process_time > 2.0:  # Log requests that take more than 2 seconds
            logging.warning(f"Slow request: {request.method} {request.url.path} took {process_time:.2f}s")

        return response
    except Exception as e:
        process_time = time.time() - start_time
        # Record error
        metrics_service.record_error(request.url.path, request.method, type(e).__name__)
        # Add response time header even for errors
        response = JSONResponse(
            status_code=500,
            content={"detail": "Internal server error"}
        )
        response.headers["X-Process-Time"] = f"{process_time:.3f}"
        return response

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "timestamp": "2025-12-09T00:00:00Z"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("PORT", 8000)))