"""
FastAPI Backend for RAG Chatbot

Exposes REST API endpoints connecting the RAG agent to the Docusaurus frontend.
Supports book-wide queries, selected-text context queries, and graceful error handling.

Usage:
    uvicorn api:app --reload --host 0.0.0.0 --port 8000
"""

import logging
import os
import time
from typing import Optional, Literal

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field, constr, field_validator

# Import existing agent from Spec 3
from agent import AgentError, RetrievalError, AgentResponse as AgentResponseInternal
import agent as agent_module


# =============================================================================
# Configuration (T009)
# =============================================================================

# Load environment variables
load_dotenv()

# Logging configuration
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
logging.basicConfig(
    level=LOG_LEVEL,
    format='{"time": "%(asctime)s", "level": "%(levelname)s", "message": "%(message)s"}',
    datefmt="%Y-%m-%dT%H:%M:%S"
)
logger = logging.getLogger(__name__)


# =============================================================================
# Custom Exceptions (T007)
# =============================================================================

class ValidationError(Exception):
    """Raised when request validation fails."""
    pass


# Note: AgentError and RetrievalError are imported from agent.py


# =============================================================================
# Pydantic Data Models (T005)
# =============================================================================

class ChatRequest(BaseModel):
    """Request model for chat endpoint (FR-001, FR-003)."""
    query: constr(min_length=1, max_length=2000, strip_whitespace=True)  # type: ignore
    selected_text: Optional[str] = None
    module_filter: Optional[Literal["intro", "ros2", "simulation", "isaac", "vla"]] = None
    top_k: int = Field(default=5, ge=1, le=100)

    @field_validator('selected_text')
    @classmethod
    def validate_selected_text(cls, v):
        """Validate selected_text length (max 5000 chars per data-model.md)."""
        if v and len(v) > 5000:
            raise ValueError("Selected text too long (max 5000 characters)")
        return v

    @field_validator('query')
    @classmethod
    def validate_query(cls, v):
        """Ensure query is not empty after stripping."""
        if not v or not v.strip():
            raise ValueError("Query cannot be empty")
        return v.strip()


class Citation(BaseModel):
    """Citation model for source references (FR-004)."""
    page_title: str
    page_url: str
    module_name: str


class ErrorDetail(BaseModel):
    """Error detail model for structured error responses (FR-006)."""
    code: str
    message: str
    suggestion: Optional[str] = None


class ChatResponse(BaseModel):
    """Response model for chat endpoint (FR-004, FR-013)."""
    answer: str
    citations: list[Citation]
    confidence: Literal["high", "low", "none"]
    retrieval_time_ms: float
    generation_time_ms: float
    total_time_ms: float
    error: Optional[ErrorDetail] = None


class ErrorResponse(BaseModel):
    """Wrapper for error responses."""
    error: ErrorDetail


class HealthStatus(BaseModel):
    """Health status model for monitoring endpoints (FR-014)."""
    status: Literal["ready", "not ready", "degraded"]
    checks: dict[str, bool]
    version: str


# =============================================================================
# FastAPI Application (T006)
# =============================================================================

app = FastAPI(
    title="RAG Chatbot API",
    description="FastAPI backend for Physical AI & Robotics book RAG chatbot",
    version="1.0.0",
)

# CORS Middleware (FR-007)
CORS_ORIGINS = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGINS,
    allow_credentials=False,  # stateless, no cookies (FR-008)
    allow_methods=["POST", "GET"],
    allow_headers=["Content-Type"],
)


# =============================================================================
# Exception Handlers (T008)
# =============================================================================

@app.exception_handler(ValidationError)
async def validation_exception_handler(request, exc):
    """Handle validation errors with user-friendly messages (FR-006, SC-003)."""
    logger.warning(f"Validation error: {str(exc)}")
    return JSONResponse(
        status_code=status.HTTP_400_BAD_REQUEST,
        content={
            "error": {
                "code": "VALIDATION_ERROR",
                "message": str(exc),
                "suggestion": "Please check your input and try again"
            }
        }
    )


@app.exception_handler(AgentError)
async def agent_exception_handler(request, exc):
    """Handle agent errors (OpenAI API failures) (FR-006, FR-012)."""
    logger.error(f"Agent error: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={
            "error": {
                "code": "AGENT_ERROR",
                "message": "Unable to generate a response. Please try again.",
                "suggestion": "If the problem persists, contact support"
            }
        }
    )


@app.exception_handler(RetrievalError)
async def retrieval_exception_handler(request, exc):
    """Handle retrieval errors (Qdrant connection failures) (FR-006, FR-012)."""
    logger.error(f"Retrieval error: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
        content={
            "error": {
                "code": "RETRIEVAL_ERROR",
                "message": "Search service is temporarily unavailable",
                "suggestion": "Please try again in a few moments"
            }
        }
    )


@app.exception_handler(ValueError)
async def value_error_handler(request, exc):
    """Handle Pydantic validation errors."""
    logger.warning(f"Value error: {str(exc)}")
    return JSONResponse(
        status_code=status.HTTP_400_BAD_REQUEST,
        content={
            "error": {
                "code": "VALIDATION_ERROR",
                "message": str(exc),
                "suggestion": "Please check your input and try again"
            }
        }
    )


# =============================================================================
# Health Endpoints (T010, T011)
# =============================================================================

@app.get("/health")
async def health():
    """
    Health check (liveness probe) (FR-014).

    Returns 200 if the API process is alive. Does not verify external dependencies.
    Used by container orchestrators for liveness checks.
    """
    return {"status": "ok"}


@app.get("/ready", response_model=HealthStatus)
async def readiness():
    """
    Readiness check (readiness probe) (FR-014, SC-008).

    Returns 200 if the API can serve traffic. Verifies agent module, Qdrant
    connection, and environment configuration. Used by load balancers to
    determine if instance should receive requests.

    Must respond within 500ms (SC-008).
    """
    start_time = time.time()

    checks = {
        "agent": False,
        "retrieval": False,
        "env": False
    }

    # Check agent module loads
    try:
        agent_config = agent_module.load_agent_config()
        checks["agent"] = bool(agent_config.openai_api_key)
    except Exception as e:
        logger.error(f"Agent check failed: {e}")

    # Check retrieval module loads
    try:
        retrieval_config = agent_module.load_retrieval_config()
        checks["retrieval"] = bool(retrieval_config.qdrant_url)
    except Exception as e:
        logger.error(f"Retrieval check failed: {e}")

    # Check required environment variables
    required_vars = ["OPENAI_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    checks["env"] = all(os.getenv(var) for var in required_vars)

    # Determine overall status
    if all(checks.values()):
        health_status = "ready"
        status_code = status.HTTP_200_OK
    elif checks["agent"] and checks["env"]:
        health_status = "degraded"
        status_code = status.HTTP_200_OK
    else:
        health_status = "not ready"
        status_code = status.HTTP_503_SERVICE_UNAVAILABLE

    elapsed_ms = (time.time() - start_time) * 1000
    logger.info(f"Readiness check completed in {elapsed_ms:.2f}ms: {health_status}")

    response = HealthStatus(
        status=health_status,
        checks=checks,
        version="1.0.0"
    )

    if status_code == status.HTTP_503_SERVICE_UNAVAILABLE:
        raise HTTPException(status_code=status_code, detail=response.model_dump())

    return response


# =============================================================================
# Chat Endpoint (User Story 1: T012-T018)
# =============================================================================

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Submit a chat query (FR-001, FR-002, FR-003, FR-004).

    Accepts a natural language query about the book content and returns
    an AI-generated answer with citations from retrieved sections.
    Supports optional selected text for contextual queries and module filtering.

    Args:
        request: ChatRequest with query, optional selected_text, module_filter, top_k

    Returns:
        ChatResponse with answer, citations, confidence, and timing metrics

    Raises:
        HTTPException: 400 for validation errors, 500 for agent errors, 503 for retrieval errors
    """
    # T018 & T025: Log incoming request (FR-009)
    log_msg = f"Chat request received: query='{request.query[:50]}...', top_k={request.top_k}, module_filter={request.module_filter}"
    if request.selected_text:
        log_msg += f", selected_text_len={len(request.selected_text)}"
    logger.info(log_msg)

    # T017: Start timing
    request_start = time.time()

    try:
        # T013: Request validation is handled by Pydantic model (FR-005, FR-011)
        # Query length (max 2000 chars) enforced by constr validator
        # Empty query validation enforced by field_validator

        # T014 & T024: Call agent.ask_async() with query and optional selected_text parameter
        agent_response = await agent_module.ask_async(
            query=request.query,
            top_k=request.top_k,
            module_filter=request.module_filter,
            selected_text=request.selected_text  # T024: Pass selected_text for User Story 2
        )

        # T015: Map AgentResponse to ChatResponse
        response = map_agent_response_to_chat_response(agent_response)

        # T017: Calculate total request time
        request_end = time.time()
        response.total_time_ms = (request_end - request_start) * 1000

        # T018: Log response (FR-009)
        logger.info(
            f"Chat response generated: confidence={response.confidence}, "
            f"citations={len(response.citations)}, total_time={response.total_time_ms:.2f}ms"
        )

        return response

    except AgentError as e:
        # T016: Handle AgentError (HTTP 500) (FR-006, FR-012)
        logger.error(f"Agent error in chat endpoint: {str(e)}", exc_info=True)
        raise

    except RetrievalError as e:
        # T016: Handle RetrievalError (HTTP 503) (FR-006, FR-012)
        logger.error(f"Retrieval error in chat endpoint: {str(e)}", exc_info=True)
        raise

    except ValueError as e:
        # T016: Handle validation errors (HTTP 400)
        logger.warning(f"Validation error in chat endpoint: {str(e)}")
        raise

    except Exception as e:
        # T016: Handle unexpected errors (HTTP 500)
        logger.error(f"Unexpected error in chat endpoint: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": {
                    "code": "INTERNAL_ERROR",
                    "message": "An unexpected error occurred",
                    "suggestion": "Please try again later"
                }
            }
        )


# =============================================================================
# Utility Functions
# =============================================================================

def map_agent_response_to_chat_response(agent_response: AgentResponseInternal) -> ChatResponse:
    """
    Map internal AgentResponse to API ChatResponse model.

    Converts between the agent.py dataclass and the FastAPI Pydantic model.
    """
    citations = [
        Citation(
            page_title=citation.page_title,
            page_url=citation.page_url,
            module_name=citation.module_name
        )
        for citation in agent_response.citations
    ]

    error_detail = None
    if agent_response.error:
        error_detail = ErrorDetail(
            code="PARTIAL_ERROR",
            message=agent_response.error
        )

    return ChatResponse(
        answer=agent_response.answer,
        citations=citations,
        confidence=agent_response.confidence,
        retrieval_time_ms=agent_response.retrieval_time_ms,
        generation_time_ms=agent_response.generation_time_ms,
        total_time_ms=agent_response.total_time_ms,
        error=error_detail
    )


# =============================================================================
# Application Startup
# =============================================================================

@app.on_event("startup")
async def startup_event():
    """Log application startup."""
    logger.info("FastAPI application starting up")
    logger.info(f"CORS origins: {CORS_ORIGINS}")
    logger.info(f"Log level: {LOG_LEVEL}")


@app.on_event("shutdown")
async def shutdown_event():
    """Log application shutdown."""
    logger.info("FastAPI application shutting down")


if __name__ == "__main__":
    import uvicorn
    host = os.getenv("API_HOST", "0.0.0.0")
    port = int(os.getenv("API_PORT", "8000"))
    uvicorn.run(app, host=host, port=port)
