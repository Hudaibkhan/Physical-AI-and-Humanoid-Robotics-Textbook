from fastapi import APIRouter
from src.services.metrics_service import metrics_service
from pydantic import BaseModel
from typing import Dict, List, Optional

router = APIRouter()


class MetricsSummaryResponse(BaseModel):
    timestamp: str
    request_counts: Dict[str, int]
    response_time_stats: Dict[str, Dict[str, float]]
    error_counts: Dict[str, int]
    requests_per_minute: int
    total_requests: int
    total_errors: int
    error_rate: float


class EndpointMetricsResponse(BaseModel):
    endpoint: str
    request_count: int
    avg_response_time: Optional[float]
    error_count: int
    last_10_requests: List[Dict]


@router.get("/summary", response_model=MetricsSummaryResponse)
async def get_metrics_summary():
    """
    Get a summary of application metrics including request counts,
    response times, and error rates.
    """
    metrics = metrics_service.get_metrics_summary()
    return MetricsSummaryResponse(**metrics)


@router.get("/endpoint", response_model=EndpointMetricsResponse)
async def get_endpoint_metrics(endpoint: str, method: str = "POST"):
    """
    Get metrics for a specific endpoint.
    """
    metrics = metrics_service.get_endpoint_metrics(endpoint, method)
    return EndpointMetricsResponse(**metrics)


@router.get("/cleanup")
async def cleanup_metrics():
    """
    Clean up old metrics data to prevent memory issues.
    """
    metrics_service.cleanup_old_data()
    return {"status": "cleanup completed"}