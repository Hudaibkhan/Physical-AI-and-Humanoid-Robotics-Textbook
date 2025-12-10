#!/usr/bin/env python3
"""
Script to upload book content to Qdrant vector database
Chunks the book content and generates embeddings for RAG functionality
"""

import asyncio
import os
import re
from typing import List, Dict
from pathlib import Path

# Add the project root to the path so we can import our modules
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.services.qdrant_service import qdrant_service
from src.services.embedding_service import embedding_service
from src.config.settings import settings


class BookChunker:
    def __init__(self, chunk_size: int = 1000, overlap: int = 100):
        self.chunk_size = chunk_size
        self.overlap = overlap

    def chunk_text(self, text: str, book_id: str) -> List[Dict]:
        """
        Split text into overlapping chunks respecting sentence boundaries where possible
        """
        # Split by paragraphs first
        paragraphs = text.split('\n\n')
        chunks = []
        chunk_id = 0

        for paragraph in paragraphs:
            if len(paragraph.strip()) == 0:
                continue

            # If paragraph is smaller than chunk size, use as is
            if len(paragraph) <= self.chunk_size:
                chunks.append({
                    "id": f"{book_id}_chunk_{chunk_id}",
                    "book_id": book_id,
                    "content": paragraph.strip(),
                    "chunk_index": chunk_id,
                    "metadata": {"type": "paragraph"}
                })
                chunk_id += 1
            else:
                # Split paragraph into smaller chunks
                sentences = re.split(r'[.!?]+', paragraph)
                current_chunk = ""

                for sentence in sentences:
                    sentence = sentence.strip()
                    if not sentence:
                        continue

                    if len(current_chunk + " " + sentence) <= self.chunk_size:
                        current_chunk += " " + sentence
                    else:
                        if current_chunk.strip():
                            chunks.append({
                                "id": f"{book_id}_chunk_{chunk_id}",
                                "book_id": book_id,
                                "content": current_chunk.strip(),
                                "chunk_index": chunk_id,
                                "metadata": {"type": "sentence_group"}
                            })
                            chunk_id += 1

                        # Start new chunk with some overlap if possible
                        current_chunk = sentence

                        # If sentence is too long, split by length
                        if len(current_chunk) > self.chunk_size:
                            while len(current_chunk) > self.chunk_size:
                                sub_chunk = current_chunk[:self.chunk_size]
                                chunks.append({
                                    "id": f"{book_id}_chunk_{chunk_id}",
                                    "book_id": book_id,
                                    "content": sub_chunk.strip(),
                                    "chunk_index": chunk_id,
                                    "metadata": {"type": "length_split"}
                                })
                                chunk_id += 1
                                current_chunk = current_chunk[self.chunk_size - self.overlap:]

                            if current_chunk.strip():
                                current_chunk = current_chunk[:self.chunk_size]

                if current_chunk.strip():
                    chunks.append({
                        "id": f"{book_id}_chunk_{chunk_id}",
                        "book_id": book_id,
                        "content": current_chunk.strip(),
                        "chunk_index": chunk_id,
                        "metadata": {"type": "sentence_group"}
                    })
                    chunk_id += 1

        return chunks


async def process_book_file(file_path: str, book_id: str):
    """Process a single book file and upload to Qdrant"""
    print(f"Processing book: {file_path}")

    # Read the book content
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Chunk the content
    chunker = BookChunker(chunk_size=1000, overlap=100)
    chunks = chunker.chunk_text(content, book_id)

    print(f"Created {len(chunks)} chunks for book {book_id}")

    # Generate embeddings for all chunks
    print("Generating embeddings...")
    texts = [chunk["content"] for chunk in chunks]

    # Process embeddings in smaller batches to avoid quota limits
    embeddings = []
    batch_size = 5  # Small batch size to stay within quota limits
    for i in range(0, len(texts), batch_size):
        batch_texts = texts[i:i+batch_size]
        try:
            batch_embeddings = await embedding_service.create_embeddings_batch(batch_texts)
            embeddings.extend(batch_embeddings)
            print(f"  Processed batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")
            # Add a small delay to avoid rate limiting
            await asyncio.sleep(1)
        except Exception as e:
            print(f"  Error processing batch: {e}")
            # Use fallback embeddings for this batch
            for _ in batch_texts:
                embeddings.append([0.0] * 3072)  # Use 3072-dim as per our test

    # Add embeddings to chunks
    for i, chunk in enumerate(chunks):
        if i < len(embeddings):
            chunk["embedding"] = embeddings[i]
        else:
            # Use fallback if embedding failed
            chunk["embedding"] = [0.0] * 3072

    # Upload to Qdrant
    print("Uploading to Qdrant...")
    success = await qdrant_service.store_chunks(chunks)

    if success:
        print(f"Successfully uploaded {len(chunks)} chunks for book {book_id}")
    else:
        print(f"Failed to upload chunks for book {book_id}")

    return success


async def main():
    """Main function to process all book files"""
    # Get book directory from environment or use default
    book_dir = os.getenv("BOOK_DIR", "../docs")  # Updated to point to the docs directory from backend
    book_path = Path(book_dir)

    if not book_path.exists():
        print(f"Book directory {book_path} does not exist!")
        return

    # Get all markdown files in the book directory and subdirectories
    book_files = list(book_path.rglob("*.md")) + list(book_path.rglob("*.markdown"))

    if not book_files:
        print(f"No markdown files found in {book_path}")
        return

    print(f"Found {len(book_files)} book files to process")

    for book_file in book_files:
        # Use the relative path to create a more descriptive book ID
        relative_path = book_file.relative_to(book_path)
        book_id = str(relative_path).replace(os.sep, '_').replace(" ", "_").replace("-", "_").replace(".md", "")

        # Limit book_id length to avoid issues with Qdrant
        if len(book_id) > 50:
            book_id = book_id[-50:]

        try:
            await process_book_file(str(book_file), book_id)
        except Exception as e:
            print(f"Error processing {book_file}: {e}")

    print("Book processing complete!")


if __name__ == "__main__":
    asyncio.run(main())