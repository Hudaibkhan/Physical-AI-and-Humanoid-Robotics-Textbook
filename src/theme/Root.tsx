/**
 * Root - Global wrapper component for Docusaurus
 *
 * This component wraps the entire Docusaurus application,
 * allowing us to add global UI elements like the chatbot and authentication context.
 */
import React from 'react';
import { AuthProvider } from '../auth/context/AuthContext';

interface RootProps {
  children: React.ReactNode;
}

export default function Root({ children }: RootProps): JSX.Element {
  const [isOpen, setIsOpen] = React.useState(false);
  const [messages, setMessages] = React.useState<any[]>([]);
  const [isLoading, setIsLoading] = React.useState(false);
  const [isThinking, setIsThinking] = React.useState(false);  // NEW: Thinking animation state
  const [selectedText, setSelectedText] = React.useState<string>('');

  const toggleOpen = () => setIsOpen(!isOpen);

  const sendMessage = async (question: string, isFromSelectedText = false) => {
    if (!question.trim()) return;

    const userMessage = {
      role: 'user' as const,
      content: question,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);
    setIsThinking(true);  // NEW: Start thinking animation

    try {
      console.log('Sending question to API:', question);

      // Note: With Better Auth's built-in session management, we don't need to manually handle auth tokens
      // The session will be managed automatically by Better Auth
      const authToken = null; // Better Auth handles this automatically

      // Get RAG backend URL from environment variable
      const RAG_BACKEND_URL = process.env.NEXT_PUBLIC_RAG_BACKEND_URL || 'http://localhost:8000';

      // Use different endpoint for selected text queries
      let apiUrl = `${RAG_BACKEND_URL}/chat`;
      let requestBody: any = {
        message: question,
        selected_text: null,
        session_id: null
      };

      if (isFromSelectedText && selectedText) {
        apiUrl = `${RAG_BACKEND_URL}/chat`;
        requestBody = {
          message: question,
          selected_text: selectedText,
          session_id: null
        };
      }

      const headers: Record<string, string> = {
        'Content-Type': 'application/json'
      };

      // Include auth token if available
      if (authToken) {
        headers['Authorization'] = `Bearer ${authToken}`;
      }

      const response = await fetch(apiUrl, {
        method: 'POST',
        headers,
        body: JSON.stringify(requestBody),
      });

      console.log('API response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('API error response:', errorText);
        throw new Error(`API error: ${response.status} - ${errorText}`);
      }

      const data = await response.json();
      console.log('API response data:', data);

      setIsThinking(false);  // NEW: Stop thinking animation on response

      setMessages((prev) => [
        ...prev,
        {
          role: 'assistant' as const,
          content: data.response || 'No response received',
          sources: data.source_chunks || [],
          timestamp: new Date(),
        },
      ]);
    } catch (err) {
      console.error('Chatbot error:', err);
      setIsThinking(false);  // NEW: Stop thinking animation on error
      const errorMessage = err instanceof Error ? err.message : 'Unknown error';
      setMessages((prev) => [
        ...prev,
        {
          role: 'assistant' as const,
          content: `❌ **Error**: ${errorMessage}\n\nPlease check the browser console for details, or try asking another question.`,
          timestamp: new Date(),
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle text selection
  React.useEffect(() => {
    const handleSelection = (e: Event) => {
      // Check if the click occurred inside the chat modal
      const target = e.target as HTMLElement;
      const chatModal = document.querySelector('.chatbot-modal');
      const isAskSelectedButton = target.closest('button')?.textContent?.includes('Ask Selected');
      const isClearButton = target.closest('button')?.textContent?.includes('Clear');

      // If the click is on the "Ask Selected" or "Clear" buttons, don't clear the selection yet
      // The actual clearing will happen after the action is processed
      if (isAskSelectedButton || isClearButton) {
        return;
      }

      // If the click is inside the chat modal (but not on action buttons), don't clear the selection
      if (chatModal && chatModal.contains(target)) {
        return;
      }

      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      } else {
        // Only clear selection if not clicking inside the chat modal or on action buttons
        if (!chatModal || !chatModal.contains(target)) {
          setSelectedText('');
        }
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', (e) => {
      if (e.key === 'Shift') {
        setTimeout(() => {
          const selectedText = window.getSelection()?.toString().trim();
          if (selectedText) {
            setSelectedText(selectedText);
          }
        }, 0);
      }
    });

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', (e) => {
        if (e.key === 'Shift') {
          setTimeout(() => {
            const selectedText = window.getSelection()?.toString().trim();
            if (selectedText) {
              setSelectedText(selectedText);
            }
          }, 0);
        }
      });
    };
  }, []);

  // NEW: ThinkingAnimation component
  const ThinkingAnimation = () => (
    <div className="thinking-container">
      <div className="thinking-bubble">
        <span className="thinking-text">Thinking</span>
        <div className="thinking-dots">
          <div className="thinking-dot"></div>
          <div className="thinking-dot"></div>
          <div className="thinking-dot"></div>
        </div>
      </div>
    </div>
  );

  return (
    <AuthProvider>
      {children}

      {/* NEW: CSS for thinking animation */}
      <style>{`
        @keyframes wave {
          0%, 100% { opacity: 0.3; }
          50% { opacity: 1; }
        }

        .thinking-container {
          display: flex;
          justify-content: flex-start;
          padding: 1rem 0;
        }

        .thinking-bubble {
          background: #f1f5f9;
          border-radius: 12px;
          padding: 1rem 1.5rem;
          display: flex;
          align-items: center;
          gap: 0.75rem;
        }

        .thinking-text {
          color: #64748b;
          font-size: 0.95rem;
        }

        .thinking-dots {
          display: flex;
          gap: 0.25rem;
        }

        .thinking-dot {
          width: 8px;
          height: 8px;
          border-radius: 50%;
          background-color: #94a3b8;
          animation: wave 1s ease-in-out infinite;
        }

        .thinking-dot:nth-child(1) { animation-delay: 0ms; }
        .thinking-dot:nth-child(2) { animation-delay: 150ms; }
        .thinking-dot:nth-child(3) { animation-delay: 300ms; }
      `}</style>

      {/* Floating chatbot icon */}
      <button
        className="chatbot-icon"
        onClick={toggleOpen}
        aria-label="Open chatbot"
        style={{
          position: 'fixed',
          bottom: '2rem',
          right: '2rem',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          background: 'linear-gradient(135deg, #1a6bf7ff 0%, #c2c4e0ff 100%)',
          border: 'none',
          boxShadow: '0 4px 20px rgba(102, 126, 234, 0.4)',
          cursor: 'pointer',
          color: 'white',
          fontSize: '24px',
          zIndex: 1000,
        }}
      >
        🗯️
      </button>

      {/* Chatbot modal */}
      {isOpen && (
        <div
          className="chatbot-overlay"
          onClick={toggleOpen}
          style={{
            position: 'fixed',
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            background: 'rgba(0, 0, 0, 0.5)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            zIndex: 2000,
          }}
        >
          <div
            className="chatbot-modal"
            onClick={(e) => e.stopPropagation()}
            style={{
              width: '90%',
              maxWidth: '600px',
              height: '80vh',
              maxHeight: '700px',
              background: 'white',
              borderRadius: '20px',
              boxShadow: '0 20px 60px rgba(0, 0, 0, 0.3)',
              display: 'flex',
              flexDirection: 'column',
              overflow: 'hidden',
            }}
          >
            {/* Header */}
            <div
              style={{
                background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                color: 'white',
                padding: '1.5rem',
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
              }}
            >
              <h3 style={{ margin: 0 }}>💬 Textbook Assistant</h3>
              <button
                onClick={toggleOpen}
                style={{
                  background: 'rgba(255, 255, 255, 0.2)',
                  border: 'none',
                  color: 'white',
                  width: '32px',
                  height: '32px',
                  borderRadius: '50%',
                  cursor: 'pointer',
                  fontSize: '1.25rem',
                }}
              >
                ✕
              </button>
            </div>

            {/* Messages */}
            <div
              style={{
                flex: 1,
                overflowY: 'auto',
                padding: '1.5rem',
                display: 'flex',
                flexDirection: 'column',
                gap: '1rem',
              }}
            >
              {messages.length === 0 && (
                <div style={{ textAlign: 'center', padding: '2rem', color: '#64748b' }}>
                  <p>👋 Hi! I'm your Physical AI textbook assistant.</p>
                  <p>Ask me anything about the course content!</p>
                </div>
              )}

              {messages.map((msg, idx) => (
                <div
                  key={idx}
                  style={{
                    display: 'flex',
                    justifyContent: msg.role === 'user' ? 'flex-end' : 'flex-start',
                  }}
                >
                  <div
                    style={{
                      maxWidth: '80%',
                      padding: '1rem 1.25rem',
                      borderRadius: '16px',
                      background:
                        msg.role === 'user'
                          ? 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)'
                          : '#f1f5f9',
                      color: msg.role === 'user' ? 'white' : '#1e293b',
                    }}
                  >
                    <p style={{ margin: 0 }}>{msg.content}</p>
                    {/* REMOVED: source_chunks display - metadata suppression (T022) */}
                  </div>
                </div>
              ))}

              {/* NEW: Show thinking animation instead of old loading indicator */}
              {isThinking && <ThinkingAnimation />}
            </div>

            {/* Input */}
            <form
              onSubmit={(e) => {
                e.preventDefault();
                const input = (e.target as any).question;
                const question = input.value;

                if (selectedText) {
                  // Use selected text functionality
                  sendMessage(question, true);
                  // Clear the selected text after sending
                  setSelectedText('');
                } else {
                  // Use standard RAG functionality
                  sendMessage(question, false);
                }

                input.value = '';
              }}
              style={{
                padding: '1.5rem',
                borderTop: '1px solid #e2e8f0',
                display: 'flex',
                flexDirection: 'column',
                gap: '0.75rem',
              }}
            >
              {selectedText && (
                <div
                  style={{
                    padding: '0.75rem',
                    background: '#e0f2fe',
                    border: '1px solid #7dd3fc',
                    borderRadius: '8px',
                    fontSize: '0.85rem',
                  }}
                >
                  <strong>Selected text:</strong> {selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}
                  <button
                    type="button"
                    onClick={() => {
                      // Send the selected text as the question
                      sendMessage("Explain " + selectedText, true);
                      // Clear the selected text after sending
                      setSelectedText('');
                    }}
                    style={{
                      background: 'none',
                      border: 'none',
                      color: '#0284c7',
                      cursor: 'pointer',
                      fontWeight: 'bold',
                      marginLeft: '10px',
                      fontSize: '0.85rem',
                    }}
                  >
                    Ask Selected
                  </button>
                  <button
                    type="button"
                    onClick={() => setSelectedText('')}
                    style={{
                      background: 'none',
                      border: 'none',
                      color: '#0284c7',
                      cursor: 'pointer',
                      marginLeft: '10px',
                      fontSize: '0.85rem',
                    }}
                  >
                    Clear
                  </button>
                </div>
              )}
              <div style={{ display: 'flex', gap: '0.75rem' }}>
                <input
                  name="question"
                  type="text"
                  placeholder={selectedText ? "Ask about selected text..." : "Ask a question..."}
                  disabled={isLoading}
                  style={{
                    flex: 1,
                    border: '2px solid #e2e8f0',
                    borderRadius: '12px',
                    padding: '0.75rem 1rem',
                    fontSize: '0.95rem',
                    outline: 'none',
                  }}
                />
                <button
                  type="submit"
                  disabled={isLoading}
                  style={{
                    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                    color: 'white',
                    border: 'none',
                    borderRadius: '12px',
                    padding: '0.75rem 1.5rem',
                    cursor: 'pointer',
                    fontWeight: 'bold',
                  }}
                >
                  {isLoading ? 'Sending...' : selectedText ? 'Send Question' : 'Send'}
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </AuthProvider>
  );
}