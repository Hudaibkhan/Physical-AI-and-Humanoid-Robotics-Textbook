import time
import threading
from datetime import datetime, timedelta
from typing import Dict, List, Optional
from collections import defaultdict, deque
from src.services.cache_service import cache_service


class MetricsService:
    def __init__(self):
        self.cache = cache_service
        self.request_counts = defaultdict(int)
        self.response_times = defaultdict(list)
        self.error_counts = defaultdict(int)
        self.lock = threading.Lock()  # Thread safety for in-memory metrics
        self.daily_metrics = defaultdict(lambda: defaultdict(int))  # daily_stats[date][metric] = value
        self.recent_requests = deque(maxlen=1000)  # Keep last 1000 requests for real-time metrics

    def record_request(self, endpoint: str, method: str = "POST"):
        """Record an incoming request"""
        key = f"{method}:{endpoint}"
        with self.lock:
            self.request_counts[key] += 1
            self.recent_requests.append({
                "endpoint": key,
                "timestamp": time.time(),
                "type": "request"
            })

    def record_response_time(self, endpoint: str, method: str, response_time: float):
        """Record response time for an endpoint"""
        key = f"{method}:{endpoint}"
        with self.lock:
            self.response_times[key].append(response_time)
            # Keep only last 100 response times for each endpoint to prevent memory issues
            if len(self.response_times[key]) > 100:
                self.response_times[key] = self.response_times[key][-100:]

    def record_error(self, endpoint: str, method: str, error_type: str = "unknown"):
        """Record an error occurrence"""
        key = f"{method}:{endpoint}:{error_type}"
        with self.lock:
            self.error_counts[key] += 1
            self.recent_requests.append({
                "endpoint": f"{method}:{endpoint}",
                "timestamp": time.time(),
                "type": "error",
                "error_type": error_type
            })

    def get_metrics_summary(self) -> Dict:
        """Get a summary of current metrics"""
        with self.lock:
            # Calculate response time statistics
            response_time_stats = {}
            for endpoint, times in self.response_times.items():
                if times:
                    response_time_stats[endpoint] = {
                        "avg": sum(times) / len(times),
                        "min": min(times),
                        "max": max(times),
                        "count": len(times)
                    }

            # Calculate requests per minute
            now = time.time()
            one_minute_ago = now - 60
            recent_requests_count = sum(1 for req in self.recent_requests if req["timestamp"] > one_minute_ago)

            return {
                "timestamp": datetime.utcnow().isoformat(),
                "request_counts": dict(self.request_counts),
                "response_time_stats": response_time_stats,
                "error_counts": dict(self.error_counts),
                "requests_per_minute": recent_requests_count,
                "total_requests": sum(self.request_counts.values()),
                "total_errors": sum(self.error_counts.values()),
                "error_rate": sum(self.error_counts.values()) / max(1, sum(self.request_counts.values())) * 100
            }

    def get_endpoint_metrics(self, endpoint: str, method: str = "POST") -> Dict:
        """Get metrics for a specific endpoint"""
        key = f"{method}:{endpoint}"
        with self.lock:
            avg_response_time = None
            if key in self.response_times and self.response_times[key]:
                times = self.response_times[key]
                avg_response_time = sum(times) / len(times)

            return {
                "endpoint": key,
                "request_count": self.request_counts.get(key, 0),
                "avg_response_time": avg_response_time,
                "error_count": sum(1 for k, v in self.error_counts.items() if k.startswith(f"{key}:")),
                "last_10_requests": [
                    req for req in list(self.recent_requests)[-10:]
                    if req["endpoint"] == key
                ]
            }

    def cleanup_old_data(self):
        """Clean up old response time data to prevent memory issues"""
        with self.lock:
            # Keep only recent data
            cutoff_time = time.time() - 3600  # 1 hour ago
            self.recent_requests = deque(
                [req for req in self.recent_requests if req["timestamp"] > cutoff_time],
                maxlen=1000
            )

            # Limit response time history
            for endpoint in self.response_times:
                self.response_times[endpoint] = self.response_times[endpoint][-50:]  # Keep last 50


# Global instance
metrics_service = MetricsService()