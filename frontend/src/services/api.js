class ApiService {
  constructor(backendUrl) {
    this.backendUrl = backendUrl || process.env.BACKEND_URL || 'http://localhost:8000';
  }

  async createEmbedding(text, model = 'gemini-embedding-001') {
    try {
      const response = await fetch(`${this.backendUrl}/api/v1/embed/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ text, model })
      });

      if (!response.ok) {
        throw new Error(`API request failed: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error creating embedding:', error);
      throw error;
    }
  }

  async searchChunks(queryEmbedding, topK = 5, bookId = null) {
    try {
      const requestBody = {
        query_embedding: queryEmbedding,
        top_k: topK
      };

      if (bookId) {
        requestBody.book_id = bookId;
      }

      const response = await fetch(`${this.backendUrl}/api/v1/search/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`Search request failed: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error searching chunks:', error);
      throw error;
    }
  }

  async askQuestion(question, selectedText = '', sessionId = null) {
    try {
      const requestBody = {
        question,
        selected_text: selectedText,
        session_id: sessionId
      };

      const response = await fetch(`${this.backendUrl}/api/v1/ask-agent/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`Ask request failed: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error asking question:', error);
      throw error;
    }
  }

  async healthCheck() {
    try {
      const response = await fetch(`${this.backendUrl}/api/v1/health/`);

      if (!response.ok) {
        throw new Error(`Health check failed: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error checking health:', error);
      throw error;
    }
  }

  async createSession() {
    try {
      const response = await fetch(`${this.backendUrl}/api/v1/sessions/create`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      if (!response.ok) {
        throw new Error(`Create session failed: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error creating session:', error);
      throw error;
    }
  }

  async getConversationHistory(sessionId, limit = 10) {
    try {
      const response = await fetch(`${this.backendUrl}/api/v1/sessions/history`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ session_id: sessionId, limit })
      });

      if (!response.ok) {
        throw new Error(`Get history failed: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error getting conversation history:', error);
      throw error;
    }
  }

  async extendSession(sessionId) {
    try {
      const response = await fetch(`${this.backendUrl}/api/v1/sessions/extend`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ session_id: sessionId })
      });

      if (!response.ok) {
        throw new Error(`Extend session failed: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error extending session:', error);
      throw error;
    }
  }
}

// Export a singleton instance
const apiService = new ApiService();
export default apiService;