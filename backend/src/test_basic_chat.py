"""
Basic test script to verify the RAG chatbot functionality
"""
import asyncio
import sys
import os
from pathlib import Path

# Add the backend/src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
load_dotenv()  # Load environment variables

from src.agents.rag_agent import get_rag_agent
from src.utils.qdrant_client import get_qdrant_client
from src.config import get_config

async def test_basic_chat():
    """
    Test basic chat functionality with sample queries
    """
    print("Starting basic chat functionality test...")

    try:
        # Get required components
        rag_agent = get_rag_agent()
        config = get_config()

        print(f"SUCCESS: RAG agent initialized")
        print(f"SUCCESS: Using Qdrant collection: {config.QDRANT_COLLECTION_NAME}")

        # Test 1: Check if Qdrant collection exists
        qdrant_client = get_qdrant_client()
        try:
            collection_info = qdrant_client.get_collection(config.QDRANT_COLLECTION_NAME)
            print(f"SUCCESS: Qdrant collection exists with {collection_info.points_count} points")

            if collection_info.points_count == 0:
                print("WARNING: Collection exists but is empty - embeddings need to be generated")
        except Exception as e:
            print(f"ERROR: Qdrant collection error: {e}")
            print("HINT: Run the embedding pipeline first: python -m src.scripts.embed_pipeline <path_to_docs>")
            return False

        # Test 2: Sample query (this will work if embeddings exist)
        sample_query = "What is this textbook about?"
        print(f"\nTesting query: '{sample_query}'")

        result = await rag_agent.process_query(query=sample_query, max_chunks=3)

        print(f"SUCCESS: Query processed successfully")
        print(f"ANSWER: {result['answer'][:200]}{'...' if len(result['answer']) > 200 else ''}")
        print(f"SOURCES: {len(result['sources'])} chunks used")

        if result.get('error'):
            print(f"WARNING: Result had error: {result['error']}")

        # Test 3: Another sample query
        sample_query2 = "Give me a summary of the main concepts"
        print(f"\nTesting query: '{sample_query2}'")

        result2 = await rag_agent.process_query(query=sample_query2, max_chunks=2)

        print(f"SUCCESS: Query processed successfully")
        print(f"ANSWER: {result2['answer'][:200]}{'...' if len(result2['answer']) > 200 else ''}")
        print(f"SOURCES: {len(result2['sources'])} chunks used")

        if result2.get('error'):
            print(f"WARNING: Result had error: {result2['error']}")

        print(f"\nSUCCESS: Basic chat functionality test completed successfully!")
        return True

    except Exception as e:
        print(f"ERROR: Test failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_basic_chat())
    sys.exit(0 if success else 1)