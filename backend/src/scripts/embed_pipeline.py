#!/usr/bin/env python3
"""
Embedding pipeline script to process textbook content and store embeddings in Qdrant
"""

import sys
import os
import argparse
from pathlib import Path

# Add the backend/src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.qdrant_client import init_qdrant_collection, get_qdrant_client
from utils.embedding_utils import get_embedding_utils
from utils.document_loader import load_documents
from utils.text_chunker import chunk_text
from config import get_config
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def run_embedding_pipeline(source_path: str, force_recreate: bool = False):
    """
    Run the complete embedding pipeline

    Args:
        source_path: Path to the source documents to process
        force_recreate: Whether to recreate embeddings even if they exist
    """
    logger.info(f"Starting embedding pipeline for: {source_path}")

    try:
        # Load configuration
        config = get_config()

        # Initialize Qdrant collection
        logger.info(f"Initializing Qdrant collection: {config.QDRANT_COLLECTION_NAME}")
        init_qdrant_collection(config.QDRANT_COLLECTION_NAME)

        # Load documents
        logger.info("Loading documents...")
        documents = load_documents(source_path)
        logger.info(f"Loaded {len(documents)} documents")

        if not documents:
            logger.warning("No documents found to process")
            return

        # Process each document
        qdrant_client = get_qdrant_client()
        embedding_utils = get_embedding_utils()

        total_chunks = 0
        for doc_path, content in documents.items():
            logger.info(f"Processing document: {doc_path}")

            # Chunk the document content
            chunks = chunk_text(content, chunk_size=config.CHUNK_SIZE)
            logger.info(f"Chunked document into {len(chunks)} chunks")

            # Generate embeddings for chunks
            chunk_texts = [chunk["text"] for chunk in chunks]
            logger.info(f"Generating embeddings for {len(chunk_texts)} chunks...")

            embeddings = embedding_utils.embed_text(chunk_texts)
            logger.info(f"Generated {len(embeddings)} embeddings")

            # Prepare points for Qdrant
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                point = {
                    "id": hash(f"{Path(doc_path).name}_chunk_{i}") % (10**9),  # Use integer ID
                    "vector": embedding,
                    "payload": {
                        "content": chunk["text"],
                        "source_file": str(doc_path),
                        "chunk_index": i,
                        "metadata": {
                            "start_pos": chunk["start_pos"],
                            "end_pos": chunk["end_pos"]
                        }
                    }
                }
                points.append(point)

            # Upsert points to Qdrant
            logger.info(f"Upserting {len(points)} points to Qdrant...")
            qdrant_client.upsert(
                collection_name=config.QDRANT_COLLECTION_NAME,
                points=points
            )

            total_chunks += len(chunks)
            logger.info(f"Completed processing document: {doc_path}")

        logger.info(f"Pipeline completed successfully! Processed a total of {total_chunks} chunks.")
        print(f"\nEmbedding pipeline completed successfully!")
        print(f"Processed {len(documents)} documents")
        print(f"Created {total_chunks} chunks with embeddings")
        print(f"Stored in Qdrant collection: {config.QDRANT_COLLECTION_NAME}")

    except Exception as e:
        logger.error(f"Error in embedding pipeline: {str(e)}")
        print(f"\nError in embedding pipeline: {str(e)}")
        raise

def main():
    parser = argparse.ArgumentParser(description="Run the embedding pipeline to process textbook content")
    parser.add_argument(
        "source_path",
        help="Path to the directory containing markdown files to process"
    )
    parser.add_argument(
        "--force-recreate",
        action="store_true",
        help="Recreate embeddings even if they already exist"
    )

    args = parser.parse_args()

    # Validate source path
    if not os.path.exists(args.source_path):
        print(f"❌ Error: Source path does not exist: {args.source_path}")
        sys.exit(1)

    # Run the pipeline
    run_embedding_pipeline(args.source_path, args.force_recreate)

if __name__ == "__main__":
    main()