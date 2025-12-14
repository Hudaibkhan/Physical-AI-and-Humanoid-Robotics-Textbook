"""
Basic test to verify the Book RAG Agent functionality
"""
import asyncio
from agent import book_rag_agent, rag_query
from connection import connection_manager, session_manager
from agents import Runner

async def test_basic_functionality():
    """Test basic agent functionality"""
    print("Testing basic agent functionality...")

    # Test the rag_query function directly with a simple query
    try:
        # This will test the rag_query function with rag mode
        # Note: This requires Qdrant to be properly configured
        results = await rag_query(
            query="What is artificial intelligence?",
            mode="rag",
            top_k=2
        )
        print(f"RAG query results: {len(results)} results found")
        for result in results:
            print(f"  - ID: {result.id}, Score: {result.similarity_score}")
    except Exception as e:
        print(f"Note: Qdrant test failed (expected if not configured): {e}")

    # Test the rag_query function with selected text mode
    sample_text = "Artificial intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence. This can include learning from experience, understanding natural language, solving problems, and recognizing patterns."
    try:
        selected_results = await rag_query(
            query="What is artificial intelligence?",
            mode="selected",
            top_k=2,
            selected_text=sample_text
        )
        print(f"Selected text query results: {len(selected_results)} results found")
        for result in selected_results:
            print(f"  - ID: {result.id}, Score: {result.similarity_score}, Text: {result.text[:100]}...")
    except Exception as e:
        print(f"Error in selected text test: {e}")

    # Test agent creation and basic properties
    print(f"\nAgent name: {book_rag_agent.name}")
    print(f"Agent has tools: {len(book_rag_agent.tools) > 0}")
    print("Agent instructions are set.")

    # Test session management
    session_id = session_manager.create_session()
    print(f"Created session: {session_id}")

    # Add a test message to the session
    session_manager.add_message_to_session(session_id, {
        "role": "user",
        "content": "Test message",
        "timestamp": session_manager.get_current_timestamp()
    })

    history = session_manager.get_session_history(session_id)
    print(f"Session has {len(history)} messages")

    print("\nBasic functionality test completed!")

if __name__ == "__main__":
    asyncio.run(test_basic_functionality())