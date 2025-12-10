# Security Guide for RAG Chatbot

This document outlines the security measures implemented in the RAG Chatbot system and provides best practices for maintaining security in production environments.

## Table of Contents
- [Security Architecture](#security-architecture)
- [Implemented Security Measures](#implemented-security-measures)
- [API Security](#api-security)
- [Data Protection](#data-protection)
- [Authentication and Authorization](#authentication-and-authorization)
- [Input Validation and Sanitization](#input-validation-and-sanitization)
- [Security Headers](#security-headers)
- [Rate Limiting](#rate-limiting)
- [Monitoring and Logging](#monitoring-and-logging)
- [Security Best Practices](#security-best-practices)
- [Vulnerability Management](#vulnerability-management)

## Security Architecture

The RAG Chatbot system implements a defense-in-depth approach with multiple layers of security controls:

1. **Network Layer**: CORS policies, trusted host middleware
2. **Application Layer**: Input validation, rate limiting, security headers
3. **Data Layer**: Secure API key handling, data encryption
4. **Infrastructure Layer**: Environment isolation, access controls

## Implemented Security Measures

### Security Headers
The application implements the following security headers:

- `X-Content-Type-Options: nosniff` - Prevents MIME type sniffing
- `X-XSS-Protection: 1; mode=block` - Enables browser XSS protection
- `X-Frame-Options: DENY` - Prevents clickjacking attacks
- `Strict-Transport-Security` - Enforces HTTPS connections
- `Referrer-Policy: strict-origin-when-cross-origin` - Controls referrer information
- `Permissions-Policy` - Restricts browser features

### Rate Limiting
- API endpoints are protected with rate limiting
- Default: 10 requests per minute per IP for ask-agent endpoint
- Configurable limits for different endpoints

### Input Validation
- All API inputs are validated using Pydantic models
- Length restrictions on text inputs
- Type checking and format validation

## API Security

### Authentication
The current implementation does not include API key authentication but can be extended:

```python
# Example of how to add API key authentication
from fastapi.security import HTTPBearer
from fastapi import Depends, HTTPException

security = HTTPBearer()

def validate_api_key(api_key: str = Security(security)):
    if api_key.credentials != os.getenv("API_KEY"):
        raise HTTPException(status_code=401, detail="Invalid API key")
```

### Secure API Endpoints
- Use HTTPS in production
- Validate all input parameters
- Implement proper error handling without information disclosure
- Sanitize responses to prevent data leakage

### Request Size Limits
- Maximum request size: 10MB
- Maximum question length: 5000 characters
- Maximum text selection length: 10000 characters

## Data Protection

### API Key Management
- Store API keys in environment variables
- Never hardcode API keys in source code
- Use different keys for development and production
- Rotate keys regularly

### Data Encryption
- Data in transit: TLS 1.2+ (enforced by HTTPS)
- Data at rest: Depends on Qdrant and Redis configuration
- Sensitive data: Use encryption libraries for additional protection

### Data Retention
- Cache entries expire automatically (configurable TTL)
- Session data has limited lifetime
- Consider implementing data retention policies for logs

## Authentication and Authorization

### Current Implementation
- No built-in user authentication (stateless API)
- IP-based rate limiting for abuse prevention
- Origin-based CORS restrictions

### Recommended Enhancements
For production deployments, consider implementing:

1. **JWT-based Authentication**:
   ```python
   # Example JWT implementation
   from datetime import datetime, timedelta
   import jwt

   def create_access_token(data: dict):
       to_encode = data.copy()
       expire = datetime.utcnow() + timedelta(minutes=30)
       to_encode.update({"exp": expire})
       return jwt.encode(to_encode, SECRET_KEY, algorithm="HS256")
   ```

2. **API Key Authentication** for client applications
3. **OAuth 2.0** for user authentication if needed

## Input Validation and Sanitization

### Request Validation
All API requests are validated using Pydantic models with the following constraints:

- Text length limits
- Format validation
- Type checking
- Required field validation

### Sanitization
- User inputs are not directly executed as code
- Database queries use parameterized statements
- HTML content is properly escaped where applicable

### Content Security
- Text selection content is treated as context, not executable code
- No server-side code execution based on user input
- Context is properly isolated from system functions

## Security Headers

### Configuration
Security headers are configured via the `SecuritySettings` class:

```python
class SecuritySettings(BaseSettings):
    security_content_type_nosniff: bool = True
    security_xss_protection: bool = True
    security_frame_deny: bool = True
    security_hsts_max_age: int = 31536000  # 1 year
    security_hsts_include_subdomains: bool = True
    security_hsts_preload: bool = False
```

### Production Considerations
- Enable HSTS only with proper HTTPS setup
- Update allowed hosts for TrustedHost middleware
- Configure proper CORS origins (not wildcard in production)

## Rate Limiting

### Current Implementation
- Uses `slowapi` library for rate limiting
- Different limits for different endpoints
- IP-based tracking

### Configuration
```python
# Example rate limiting configuration
@router.post("/", response_model=AskAgentResponse)
@limiter.limit("10/minute")  # 10 requests per minute per IP
async def ask_agent(request: AskAgentRequest):
    # endpoint implementation
```

### Best Practices
- Monitor rate limit violations for potential abuse
- Adjust limits based on usage patterns
- Consider implementing account-based limits in addition to IP-based

## Monitoring and Logging

### Security Logging
The application includes security-relevant logging:

- Authentication attempts (when implemented)
- Rate limit violations
- Suspicious request patterns
- Error details without sensitive information disclosure

### Metrics
- Request counts and response times
- Error rates and types
- Resource usage patterns

### Monitoring Recommendations
- Set up alerts for unusual traffic patterns
- Monitor for repeated failed requests
- Track API usage for billing and abuse prevention

## Security Best Practices

### Development
- Use environment variables for sensitive configuration
- Never commit secrets to version control
- Regular security updates for dependencies
- Code reviews with security focus

### Deployment
- Use HTTPS with valid certificates
- Configure proper CORS policies
- Implement network segmentation
- Regular security scans

### Operations
- Monitor logs for suspicious activity
- Regular security assessments
- Incident response procedures
- Backup and recovery testing

## Vulnerability Management

### Dependency Security
- Regular updates of dependencies
- Security scanning of packages
- Pin dependency versions in production
- Monitor for known vulnerabilities

### Security Updates
- Subscribe to security mailing lists
- Regular security assessments
- Penetration testing
- Bug bounty programs (if applicable)

### Incident Response
1. Detection and analysis
2. Containment and eradication
3. Recovery and post-incident activities
4. Documentation and lessons learned

## Environment-Specific Configurations

### Production Security Settings
```env
# Production-specific security settings
DEBUG=False
SECURITY_HSTS_PRELOAD=True
CORS_ALLOW_ORIGINS=["https://yourdomain.com", "https://www.yourdomain.com"]
ALLOWED_HOSTS=["yourdomain.com", "www.yourdomain.com"]
```

### Development vs Production
- Different CORS policies
- More permissive settings in development
- Enhanced logging in development
- Strict validation in production

## Conclusion

Security is an ongoing process that requires continuous attention and updates. This guide provides a foundation for securing the RAG Chatbot system, but security practices should be regularly reviewed and updated based on new threats and best practices.

Regular security assessments, keeping dependencies updated, and monitoring for suspicious activity are essential for maintaining a secure system.