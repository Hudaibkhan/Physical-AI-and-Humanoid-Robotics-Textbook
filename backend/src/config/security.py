from pydantic_settings import BaseSettings
from typing import List, Optional


class SecuritySettings(BaseSettings):
    # Security headers
    security_content_type_nosniff: bool = True
    security_xss_protection: bool = True
    security_frame_deny: bool = True
    security_hsts_max_age: int = 31536000  # 1 year
    security_hsts_include_subdomains: bool = True
    security_hsts_preload: bool = False

    # Rate limiting
    rate_limit_requests: int = 100  # requests per minute per IP
    rate_limit_window: int = 60  # seconds

    # CORS settings
    cors_allow_credentials: bool = True
    cors_allow_origins: List[str] = ["*"]  # Should be configured properly in production
    cors_allow_methods: List[str] = ["*"]
    cors_allow_headers: List[str] = ["*"]

    # API security
    api_request_timeout: int = 30  # seconds
    api_max_request_size: int = 10 * 1024 * 1024  # 10MB

    # Authentication
    jwt_secret_key: Optional[str] = None
    jwt_algorithm: str = "HS256"
    jwt_access_token_expire_minutes: int = 30

    # Input validation
    max_question_length: int = 5000
    max_text_selection_length: int = 10000
    max_embedding_text_length: int = 10000

    # Security scanning
    enable_security_scanning: bool = True
    security_scan_timeout: int = 10  # seconds

    class Config:
        # No env file needed for security settings as they have default values
        pass


security_settings = SecuritySettings()