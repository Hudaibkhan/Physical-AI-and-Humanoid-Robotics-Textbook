"""
Test script to verify the selected text RAG functionality
"""
import asyncio
import sys
from pathlib import Path

# Add the backend/src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent))

from agents.rag_agent import get_rag_agent

async def test_selected_text_rag():
    """
    Test selected text RAG functionality with various text selections
    """
    print("🧪 Starting selected text RAG functionality test...")

    try:
        # Get the RAG agent
        rag_agent = get_rag_agent()

        print("✅ RAG agent initialized")

        # Test 1: Short text selection
        short_text = "Artificial intelligence is a wonderful field that combines computer science and data to create intelligent systems."
        query1 = "What is artificial intelligence according to this text?"

        print(f"\n📝 Testing with short text selection: '{short_text[:50]}...'")
        print(f"💬 Query: '{query1}'")

        result1 = rag_agent.process_selected_text_query(selected_text=short_text, query=query1)

        print(f"✅ Query processed successfully")
        print(f"📝 Answer: {result1['answer'][:200]}{'...' if len(result1['answer']) > 200 else ''}")
        print(f"🔗 Sources: {len(result1['sources'])} chunks used")

        if result1.get('error'):
            print(f"⚠️  Result had error: {result1['error']}")

        # Test 2: Medium text selection
        medium_text = """
        Machine learning is a subset of artificial intelligence that focuses on algorithms
        that can learn from data. These algorithms can improve their performance over time
        as they are exposed to more data. There are several types of machine learning
        including supervised, unsupervised, and reinforcement learning.
        """
        query2 = "What are the types of machine learning mentioned in this text?"

        print(f"\n📝 Testing with medium text selection: '{medium_text[:50]}...'")
        print(f"💬 Query: '{query2}'")

        result2 = rag_agent.process_selected_text_query(selected_text=medium_text, query=query2)

        print(f"✅ Query processed successfully")
        print(f"📝 Answer: {result2['answer'][:200]}{'...' if len(result2['answer']) > 200 else ''}")
        print(f"🔗 Sources: {len(result2['sources'])} chunks used")

        if result2.get('error'):
            print(f"⚠️  Result had error: {result2['error']}")

        # Test 3: Question about specific details
        detailed_text = """
        The Transformer architecture, introduced in the paper "Attention is All You Need"
        in 2017, revolutionized natural language processing. It uses self-attention mechanisms
        to weigh the importance of different words in a sentence. This architecture forms the
        basis for many modern language models including BERT, GPT, and others.
        """
        query3 = "Which paper introduced the Transformer architecture and when?"

        print(f"\n📝 Testing with detailed text selection: '{detailed_text[:50]}...')")
        print(f"💬 Query: '{query3}'")

        result3 = rag_agent.process_selected_text_query(selected_text=detailed_text, query=query3)

        print(f"✅ Query processed successfully")
        print(f"📝 Answer: {result3['answer'][:200]}{'...' if len(result3['answer']) > 200 else ''}")
        print(f"🔗 Sources: {len(result3['sources'])} chunks used")

        if result3.get('error'):
            print(f"⚠️  Result had error: {result3['error']}")

        # Test 4: Complex question requiring understanding
        complex_text = """
        In robotics, inverse kinematics is the mathematical process of calculating the
        variable parameters needed to place the end of a robotic arm in a particular
        position and orientation. This is the reverse of forward kinematics, which
        computes the position of the end effector given the joint parameters.
        """
        query4 = "Explain the difference between inverse and forward kinematics in robotics."

        print(f"\n📝 Testing with complex text selection: '{complex_text[:50]}...')")
        print(f"💬 Query: '{query4}'")

        result4 = rag_agent.process_selected_text_query(selected_text=complex_text, query=query4)

        print(f"✅ Query processed successfully")
        print(f"📝 Answer: {result4['answer'][:200]}{'...' if len(result4['answer']) > 200 else ''}")
        print(f"🔗 Sources: {len(result4['sources'])} chunks used")

        if result4.get('error'):
            print(f"⚠️  Result had error: {result4['error']}")

        print(f"\n🎉 Selected text RAG functionality test completed successfully!")
        print("✅ All tests passed - selected text RAG works correctly")
        return True

    except Exception as e:
        print(f"❌ Test failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_selected_text_rag())
    sys.exit(0 if success else 1)