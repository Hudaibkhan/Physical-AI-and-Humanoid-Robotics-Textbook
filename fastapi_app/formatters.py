"""Response formatting and metadata suppression for agent behavior enhancement.

This module provides utilities to clean RAG metadata from responses and validate
that responses maintain professional quality without exposing internal system details.
"""

import re
from typing import List


class ResponseFormatter:
    """Cleans and formats agent responses to suppress RAG metadata."""

    # Regex patterns that indicate RAG metadata leakage
    METADATA_PATTERNS: List[str] = [
        r'Source:\s*chunk_\w+',          # "Source: chunk_123"
        r'Based on chunk \w+',            # "Based on chunk abc"
        r'According to document \w+',     # "According to document xyz"
        r'\[Retrieved from:.*?\]',        # "[Retrieved from: ...]"
        r'Chunk ID:.*',                   # "Chunk ID: anything"
        r'Similarity score:.*',           # "Similarity score: 0.95"
        r'chunk_\d+',                     # "chunk_123"
        r'document_\d+',                  # "document_456"
    ]

    @classmethod
    def strip_metadata(cls, text: str) -> str:
        """
        Remove RAG metadata from response text.

        Applies regex patterns to remove internal references like chunk IDs,
        source annotations, and similarity scores. Also removes excessive
        whitespace left after metadata removal.

        Args:
            text: Response text possibly containing metadata

        Returns:
            Cleaned text without metadata references

        Examples:
            >>> ResponseFormatter.strip_metadata("Answer here. Source: chunk_123")
            'Answer here.'
            >>> ResponseFormatter.strip_metadata("Based on chunk abc, the answer is...")
            'the answer is...'
        """
        if not text:
            return text

        cleaned = text

        # Apply all metadata removal patterns
        for pattern in cls.METADATA_PATTERNS:
            cleaned = re.sub(pattern, '', cleaned, flags=re.IGNORECASE)

        # Remove excessive whitespace (3+ newlines → 2 newlines)
        cleaned = re.sub(r'\n{3,}', '\n\n', cleaned)

        # Remove leading/trailing whitespace
        cleaned = cleaned.strip()

        return cleaned

    @classmethod
    def validate_no_metadata(cls, text: str) -> bool:
        """
        Check if text contains forbidden metadata patterns.

        Useful for validation and logging warnings when metadata leakage
        is detected after stripping.

        Args:
            text: Text to validate

        Returns:
            True if clean (no metadata found), False if metadata detected

        Examples:
            >>> ResponseFormatter.validate_no_metadata("Clean response here")
            True
            >>> ResponseFormatter.validate_no_metadata("Response with chunk_123")
            False
        """
        if not text:
            return True

        # Check each pattern
        for pattern in cls.METADATA_PATTERNS:
            if re.search(pattern, text, re.IGNORECASE):
                return False

        return True
