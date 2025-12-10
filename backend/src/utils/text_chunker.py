import re
from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)

def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[Dict[str, Any]]:
    """
    Split text into chunks of specified size with overlap

    Args:
        text: Text to be chunked
        chunk_size: Maximum size of each chunk (in characters)
        overlap: Number of characters to overlap between chunks

    Returns:
        List of dictionaries containing chunk text and position information
    """
    if not text:
        return []

    # Split text into sentences to avoid breaking in the middle of sentences
    sentences = re.split(r'(?<=[.!?]) +', text)

    chunks = []
    current_chunk = ""
    start_pos = 0

    for i, sentence in enumerate(sentences):
        # If adding this sentence would exceed chunk size
        if len(current_chunk) + len(sentence) > chunk_size and current_chunk:
            # Save the current chunk
            chunks.append({
                "text": current_chunk.strip(),
                "start_pos": start_pos,
                "end_pos": start_pos + len(current_chunk)
            })

            # Start new chunk with overlap from the previous chunk
            if overlap > 0:
                # Find overlap text from the end of current chunk
                overlap_text = current_chunk[-overlap:] if len(current_chunk) >= overlap else current_chunk
                current_chunk = overlap_text + " " + sentence
                start_pos = start_pos + len(current_chunk) - len(overlap_text)
            else:
                current_chunk = sentence
                start_pos = start_pos + len(chunks[-1]["text"]) if chunks else 0
        else:
            # Add sentence to current chunk
            if current_chunk:
                current_chunk += " " + sentence
            else:
                current_chunk = sentence

    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append({
            "text": current_chunk.strip(),
            "start_pos": start_pos,
            "end_pos": start_pos + len(current_chunk)
        })

    logger.info(f"Text chunked into {len(chunks)} chunks")
    return chunks

def chunk_by_tokens(text: str, max_tokens: int = 128, token_overlap: int = 10) -> List[Dict[str, Any]]:
    """
    Split text into chunks based on token count (approximation using words)

    Args:
        text: Text to be chunked
        max_tokens: Maximum number of tokens (words) per chunk
        token_overlap: Number of tokens to overlap between chunks

    Returns:
        List of dictionaries containing chunk text and position information
    """
    if not text:
        return []

    # Split text into words
    words = text.split()
    chunks = []
    start_pos = 0

    i = 0
    while i < len(words):
        # Determine the end position for this chunk
        end_pos = min(i + max_tokens, len(words))

        # Create the chunk text
        chunk_words = words[i:end_pos]
        chunk_text = " ".join(chunk_words)

        chunks.append({
            "text": chunk_text,
            "start_pos": start_pos + i,
            "end_pos": start_pos + end_pos
        })

        # Move to the next chunk position, accounting for overlap
        if end_pos < len(words):
            i = end_pos - token_overlap
        else:
            break

    logger.info(f"Text chunked into {len(chunks)} token-based chunks")
    return chunks

def chunk_by_paragraphs(text: str, max_chunk_size: int = 1024) -> List[Dict[str, Any]]:
    """
    Split text into chunks by paragraphs, combining paragraphs if they're small

    Args:
        text: Text to be chunked
        max_chunk_size: Maximum size of each chunk (in characters)

    Returns:
        List of dictionaries containing chunk text and position information
    """
    if not text:
        return []

    # Split by paragraphs (double newlines)
    paragraphs = re.split(r'\n\s*\n', text)
    chunks = []
    current_chunk = ""
    start_pos = 0

    for paragraph in paragraphs:
        paragraph = paragraph.strip()
        if not paragraph:
            continue

        # If adding this paragraph would exceed the chunk size
        if len(current_chunk) + len(paragraph) > max_chunk_size and current_chunk:
            # Save the current chunk
            chunks.append({
                "text": current_chunk.strip(),
                "start_pos": start_pos,
                "end_pos": start_pos + len(current_chunk)
            })

            # Start a new chunk with this paragraph
            current_chunk = paragraph
            start_pos = start_pos + len(chunks[-1]["text"])
        else:
            # Add paragraph to current chunk
            if current_chunk:
                current_chunk += "\n\n" + paragraph
            else:
                current_chunk = paragraph

    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append({
            "text": current_chunk.strip(),
            "start_pos": start_pos,
            "end_pos": start_pos + len(current_chunk)
        })

    logger.info(f"Text chunked into {len(chunks)} paragraph-based chunks")
    return chunks