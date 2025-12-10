#!/usr/bin/env python3
"""
Simple script to check the current settings.
"""

import sys
import os

# Add the backend directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config.settings import settings

print("Current Settings:")
print(f"  Embedding model: {settings.embedding_model}")
print(f"  Cohere API key: {'[HIDDEN]' if settings.cohere_api_key else 'NOT SET'}")
print(f"  Gemini API key: {'[HIDDEN]' if settings.gemini_api_key else 'NOT SET'}")
print(f"  Qdrant URL: {settings.qdrant_url}")
print(f"  Collection name: {settings.collection_name}")