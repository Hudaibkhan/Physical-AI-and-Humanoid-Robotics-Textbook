"""
Test script to verify the embedding pipeline functionality
"""
import asyncio
import sys
import tempfile
import os
from pathlib import Path

# Add the backend/src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent))

from utils.qdrant_client import get_qdrant_client, init_qdrant_collection
from utils.embedding_utils import get_embedding_utils
from utils.document_loader import load_documents
from utils.text_chunker import chunk_text
from config import get_config
from scripts.embed_pipeline import run_embedding_pipeline

async def test_embedding_pipeline():
    """
    Test embedding pipeline with sample textbook content
    """
    print("🧪 Starting embedding pipeline test...")

    try:
        config = get_config()
        print(f"✅ Configuration loaded")
        print(f"✅ Using Qdrant collection: {config.QDRANT_COLLECTION_NAME}")

        # Create a temporary directory with sample markdown files
        with tempfile.TemporaryDirectory() as temp_dir:
            # Create sample markdown files
            sample_content_1 = """# Introduction to Robotics

Robotics is an interdisciplinary branch of engineering and science that includes mechanical engineering, electrical engineering, computer science, and others. It deals with the design, construction, operation, and use of robots, as well as computer systems for their control, sensory feedback, and information processing.

## Types of Robots

There are various types of robots including:
- Industrial robots
- Service robots
- Medical robots
- Military robots
"""

            sample_content_2 = """# Artificial Intelligence in Robotics

Artificial intelligence (AI) is intelligence demonstrated by machines, in contrast to the natural intelligence displayed by humans and animals. Leading AI textbooks define the field as the study of "intelligent agents": any device that perceives its environment and takes actions that maximize its chance of successfully achieving its goals.

## Machine Learning

Machine learning is a subset of AI that provides systems the ability to automatically learn and improve from experience without being explicitly programmed. It focuses on the development of computer programs that can access data and use it to learn for themselves.
"""

            # Write sample files
            file1_path = os.path.join(temp_dir, "robotics_intro.md")
            file2_path = os.path.join(temp_dir, "ai_in_robotics.md")

            with open(file1_path, 'w', encoding='utf-8') as f:
                f.write(sample_content_1)

            with open(file2_path, 'w', encoding='utf-8') as f:
                f.write(sample_content_2)

            print(f"✅ Created sample markdown files in temporary directory: {temp_dir}")

            # Initialize Qdrant collection
            init_qdrant_collection(config.QDRANT_COLLECTION_NAME)
            print(f"✅ Qdrant collection initialized: {config.QDRANT_COLLECTION_NAME}")

            # Test the embedding pipeline
            print(f"🚀 Running embedding pipeline on sample content...")
            run_embedding_pipeline(temp_dir)

            # Verify the embeddings were created
            qdrant_client = get_qdrant_client()
            collection_info = qdrant_client.get_collection(config.QDRANT_COLLECTION_NAME)

            print(f"✅ Collection contains {collection_info.points_count} points after embedding")

            if collection_info.points_count == 0:
                print("❌ No embeddings were created")
                return False

            # Test retrieval to ensure embeddings work
            embedding_utils = get_embedding_utils()
            test_query = "What is robotics?"
            query_embedding = embedding_utils.embed_single_text(test_query)

            search_results = qdrant_client.search(
                collection_name=config.QDRANT_COLLECTION_NAME,
                query_vector=query_embedding,
                limit=3
            )

            print(f"✅ Successfully retrieved {len(search_results)} results for test query")

            if len(search_results) == 0:
                print("❌ No results returned from vector search")
                return False

            # Print sample results
            print(f"\n🔍 Sample search results:")
            for i, result in enumerate(search_results[:2]):  # Show first 2 results
                content_preview = result.payload.get("content", "")[:100]
                print(f"  Result {i+1}: Score={result.score:.3f}, Content='{content_preview}...'")
                print(f"    Source: {result.payload.get('source_file', 'Unknown')}")

            print(f"\n🎉 Embedding pipeline test completed successfully!")
            print(f"✅ Processed documents and created {collection_info.points_count} embeddings")
            print(f"✅ Verified embeddings work with vector search")
            return True

    except Exception as e:
        print(f"❌ Test failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_embedding_pipeline())
    sys.exit(0 if success else 1)