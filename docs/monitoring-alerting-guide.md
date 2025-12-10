# Monitoring and Alerting Guide for RAG Chatbot

This document provides comprehensive guidelines for monitoring the RAG Chatbot system and setting up appropriate alerting mechanisms.

## Table of Contents
- [Overview](#overview)
- [Key Metrics to Monitor](#key-metrics-to-monitor)
- [Application Metrics](#application-metrics)
- [Infrastructure Metrics](#infrastructure-metrics)
- [Business Metrics](#business-metrics)
- [Monitoring Tools](#monitoring-tools)
- [Alerting Configuration](#alerting-configuration)
- [Dashboard Setup](#dashboard-setup)
- [Incident Response](#incident-response)
- [Best Practices](#best-practices)

## Overview

The RAG Chatbot system requires comprehensive monitoring to ensure optimal performance, availability, and user satisfaction. This guide covers metrics collection, alerting thresholds, and monitoring best practices.

## Key Metrics to Monitor

### System Health
- **API Response Times**: P50, P95, P99 response times
- **Error Rates**: HTTP error rates and application errors
- **Availability**: System uptime and downtime
- **Throughput**: Requests per minute and concurrent users

### Application Performance
- **Qdrant Database**: Query performance, vector search times
- **Gemini API**: Latency, rate limits, error rates
- **Caching**: Hit rates, cache performance
- **Session Management**: Active sessions, session duration

### Business Metrics
- **User Engagement**: Questions asked, conversation length
- **Answer Quality**: "Not in the book" responses, confidence scores
- **Text Selection**: Usage of highlight-to-ask feature
- **User Satisfaction**: Indirect metrics through engagement

## Application Metrics

### Backend Metrics

#### Response Time Metrics
```python
# Example metrics collection in FastAPI middleware
@app.middleware("http")
async def add_process_time_header(request, call_next):
    start_time = time.time()
    metrics_service.record_request(request.url.path, request.method)

    try:
        response = await call_next(request)
        process_time = time.time() - start_time
        metrics_service.record_response_time(request.url.path, request.method, process_time)
        response.headers["X-Process-Time"] = f"{process_time:.3f}"

        return response
    except Exception as e:
        process_time = time.time() - start_time
        metrics_service.record_error(request.url.path, request.method, type(e).__name__)
        raise
```

#### Endpoint-Specific Metrics
- `/api/v1/ask-agent`: Track response times, error rates, and token usage
- `/api/v1/search`: Monitor vector search performance
- `/api/v1/embed`: Track embedding generation times
- `/api/v1/health`: System health checks

### Frontend Metrics

#### User Interaction Metrics
- **Widget Open Rate**: Percentage of users who open the chat widget
- **Message Volume**: Number of questions asked per session
- **Session Duration**: Average conversation length
- **Error Occurrences**: Frontend errors and failed API calls

#### Performance Metrics
- **Load Time**: Time to initialize the chat widget
- **Response Display Time**: Time from question submission to response display
- **Resource Usage**: JavaScript bundle size, memory usage

## Infrastructure Metrics

### Qdrant Vector Database
- **Collection Size**: Number of vectors and storage usage
- **Index Performance**: Vector search times and recall rates
- **Memory Usage**: RAM consumption for vector operations
- **Disk I/O**: Read/write performance

### Redis Cache
- **Cache Hit Rate**: Percentage of requests served from cache
- **Memory Usage**: Redis memory consumption
- **Connection Count**: Active connections to Redis
- **Eviction Rate**: Rate of cache evictions

### System Resources
- **CPU Usage**: Backend server CPU consumption
- **Memory Usage**: Application memory consumption
- **Disk Space**: Available storage space
- **Network I/O**: Request/response traffic

## Business Metrics

### User Engagement
- **Daily/Monthly Active Users**: Users interacting with the chatbot
- **Session Count**: Number of chat sessions initiated
- **Question Volume**: Total questions asked over time
- **Return Users**: Users who return to use the chatbot

### Answer Quality
- **Relevant Response Rate**: Percentage of useful responses
- **"Not in the book" Rate**: Percentage of out-of-scope questions
- **Confidence Scores**: Average confidence in responses
- **Source Citations**: Usage of source chunk references

## Monitoring Tools

### Application Performance Monitoring (APM)

#### OpenTelemetry Integration
```python
# Example OpenTelemetry setup for the backend
from opentelemetry import trace
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor

# Initialize tracer
trace.set_tracer_provider(TracerProvider())
tracer = trace.get_tracer(__name__)

# Add OTLP exporter
otlp_exporter = OTLPSpanExporter(endpoint="http://localhost:4317")
trace.get_tracer_provider().add_span_processor(
    BatchSpanProcessor(otlp_exporter)
)
```

#### Metrics Collection
- **Prometheus**: For metrics collection and storage
- **Grafana**: For dashboard visualization
- **Jaeger**: For distributed tracing

### Cloud Monitoring Services

#### For Render Deployment
- **Render Metrics**: Built-in metrics dashboard
- **External Services**: Integration with DataDog, New Relic, etc.

#### For Vercel Deployment
- **Vercel Analytics**: Frontend performance monitoring
- **Third-party**: Google Analytics, Plausible, etc.

### Custom Metrics Endpoint
The application already provides metrics at:
- `/api/v1/metrics/summary`: Overall metrics summary
- `/api/v1/metrics/endpoint`: Specific endpoint metrics

## Alerting Configuration

### Critical Alerts

#### System Availability
- **API Downtime**: Alert if health check fails for > 2 minutes
- **High Error Rate**: Alert if error rate > 5% for 5 minutes
- **Slow Response Times**: Alert if P95 response time > 5 seconds

#### Resource Exhaustion
- **Memory Usage**: Alert if memory usage > 85%
- **Disk Space**: Alert if disk space < 10% available
- **Qdrant Connection**: Alert if database connection fails

#### Service Dependencies
- **Gemini API**: Alert if API calls fail for > 5 minutes
- **Qdrant Database**: Alert if vector searches fail
- **Redis Cache**: Alert if cache operations fail

### Warning Alerts

#### Performance Degradation
- **Moderate Slowdown**: P95 response time > 3 seconds
- **Moderate Error Rate**: Error rate between 1-5%
- **Cache Miss Rate**: Cache hit rate < 70%

#### Capacity Issues
- **High CPU Usage**: CPU usage > 75%
- **High Memory Usage**: Memory usage > 75%
- **Connection Limits**: Approaching connection limits

### Alert Thresholds

#### Response Time Alerts
| Percentile | Warning | Critical |
|------------|---------|----------|
| P50 | > 1s | > 2s |
| P95 | > 3s | > 5s |
| P99 | > 5s | > 10s |

#### Error Rate Alerts
| Duration | Warning | Critical |
|----------|---------|----------|
| 1 minute | > 1% | > 5% |
| 5 minutes | > 0.5% | > 3% |
| 15 minutes | > 0.1% | > 1% |

#### Resource Usage Alerts
| Resource | Warning | Critical |
|----------|---------|----------|
| CPU | > 75% | > 90% |
| Memory | > 75% | > 90% |
| Disk | < 15% free | < 5% free |

## Dashboard Setup

### Grafana Dashboard Configuration

#### System Overview Panel
```
# API Performance
- Requests per minute
- Average response time
- Error rate percentage
- Active connections
```

#### Qdrant Database Panel
```
# Vector Database Metrics
- Query performance (ms)
- Collection size (vectors)
- Index build time
- Memory usage
```

#### Cache Performance Panel
```
# Redis Cache Metrics
- Cache hit rate (%)
- Memory usage
- Operations per second
- Connection count
```

### Dashboard Variables
- **Time Range**: 1h, 6h, 1d, 7d, 30d
- **Environment**: Development, Staging, Production
- **Service**: Backend, Frontend, Database, Cache

### Alert Dashboard
- **Active Alerts**: Currently firing alerts
- **Alert History**: Recent alert activity
- **Alert Volume**: Alerts over time

## Incident Response

### On-Call Procedures

#### Alert Triage
1. **Acknowledge alert** within 5 minutes
2. **Assess severity** based on impact
3. **Gather context** from dashboards and logs
4. **Escalate if needed** to appropriate team member

#### Common Incident Types
- **Performance Issues**: Check resource usage, Qdrant performance
- **Service Outages**: Verify dependencies, restart services if needed
- **Data Issues**: Check vector database integrity, cache status

### Runbooks

#### High Error Rate Incident
1. **Check logs** for error patterns
2. **Verify dependencies** (Qdrant, Redis, Gemini API)
3. **Review recent changes** that might cause issues
4. **Rollback if necessary** and monitor recovery

#### Slow Response Times
1. **Identify affected endpoints** using metrics
2. **Check resource usage** (CPU, memory, disk)
3. **Verify database performance** (Qdrant queries)
4. **Optimize queries** or scale resources as needed

### Communication Plan
- **Slack/Discord**: Real-time incident communication
- **Status Page**: Public status updates if needed
- **Email**: For non-critical alerts during business hours

## Best Practices

### Metric Collection
- **Use consistent naming** for metrics across services
- **Include appropriate labels** (environment, service, version)
- **Sample high-volume metrics** to reduce storage costs
- **Set appropriate retention periods** for different metric types

### Alerting
- **Avoid alert fatigue** by tuning thresholds appropriately
- **Use meaningful alert names** that indicate the issue
- **Group related alerts** to avoid notification spam
- **Test alerting configuration** regularly

### Dashboard Design
- **Focus on actionable metrics** that drive decisions
- **Use appropriate visualization** for different metric types
- **Maintain dashboard consistency** across teams
- **Update dashboards** as the system evolves

### Monitoring Strategy
- **Monitor the business** not just the technology
- **Set up synthetic monitoring** for critical user flows
- **Implement canary deployments** with monitoring
- **Regular monitoring reviews** to optimize effectiveness

## Implementation Checklist

### Backend Monitoring
- [ ] Application performance metrics collection
- [ ] Error tracking and logging
- [ ] Database performance monitoring
- [ ] Cache performance metrics
- [ ] API rate limiting monitoring
- [ ] Resource usage monitoring

### Frontend Monitoring
- [ ] Widget load performance
- [ ] API call success/failure rates
- [ ] User interaction tracking
- [ ] Error reporting
- [ ] Performance budget compliance

### Alerting Setup
- [ ] Critical alert thresholds configured
- [ ] Warning alert thresholds configured
- [ ] Alert routing to appropriate channels
- [ ] On-call rotation established
- [ ] Incident response procedures documented

### Dashboard Configuration
- [ ] System overview dashboard
- [ ] Performance metrics dashboard
- [ ] Business metrics dashboard
- [ ] Alert status dashboard
- [ ] Mobile-friendly dashboard views

## Conclusion

Effective monitoring and alerting are crucial for maintaining the reliability and performance of the RAG Chatbot system. Regular review and adjustment of metrics, thresholds, and alerting rules will ensure the system continues to meet user needs while maintaining operational excellence.