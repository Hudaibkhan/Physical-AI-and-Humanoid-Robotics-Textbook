import React, { useState, useRef, useEffect } from 'react';
import Message from './Message';
import InputArea from './InputArea';
import apiService from '../services/api';
import { getTextSelection } from '../utils/textSelection';

const ChatModal = ({ backendUrl, onClose }) => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isTyping, setIsTyping] = useState(false); // Show typing indicator when bot is processing
  const [selectedText, setSelectedText] = useState('');
  const [error, setError] = useState(null);
  const [sessionId, setSessionId] = useState(null);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Initialize session and load conversation history
  useEffect(() => {
    const initializeSession = async () => {
      try {
        // Create or get existing session ID from localStorage
        let currentSessionId = localStorage.getItem('chatbot_session_id');

        if (!currentSessionId) {
          // Create a new session
          const sessionResponse = await apiService.createSession();
          currentSessionId = sessionResponse.session_id;
          localStorage.setItem('chatbot_session_id', currentSessionId);
        }

        setSessionId(currentSessionId);

        // Load conversation history
        const historyResponse = await apiService.getConversationHistory(currentSessionId);
        const historyMessages = historyResponse.messages.map((msg, index) => ({
          id: msg.id || `msg-${index}`,
          text: msg.content,
          sender: msg.role === 'user' ? 'user' : 'bot',
          timestamp: new Date(msg.timestamp),
          context: msg.selected_text || null
        }));

        // Add welcome message if no history
        if (historyMessages.length === 0) {
          setMessages([
            { id: 1, text: "Hello! I'm your book assistant. Ask me anything about the book content.", sender: 'bot', timestamp: new Date() }
          ]);
        } else {
          setMessages(historyMessages);
        }
      } catch (err) {
        console.error('Error initializing session:', err);
        // Fallback to default welcome message
        setMessages([
          { id: 1, text: "Hello! I'm your book assistant. Ask me anything about the book content.", sender: 'bot', timestamp: new Date() }
        ]);
      }
    };

    initializeSession();
  }, []);

  // Handle text selection from the parent page
  useEffect(() => {
    // Listen for selected text
    const handleSelection = () => {
      const selectedText = getTextSelection();
      if (selectedText && selectedText.text.length > 0) {
        setSelectedText(selectedText.text);
      }
    };

    // Handle custom event when user clicks the visual indicator
    const handleCustomTextSelection = (event) => {
      const selectedText = event.detail.selectedText;
      if (selectedText && selectedText.length > 0) {
        setSelectedText(selectedText);
        // Optionally, focus the input area to encourage user to ask a question
        if (inputRef.current) {
          setTimeout(() => {
            inputRef.current.focus();
          }, 100);
        }
      }
    };

    // Add event listeners for text selection
    document.addEventListener('mouseup', handleSelection);

    // Also listen for keyup with shift key (for keyboard text selection)
    document.addEventListener('keyup', (e) => {
      if (e.key === 'Shift') {
        setTimeout(() => {
          const selectedText = getTextSelection();
          if (selectedText && selectedText.text.length > 0) {
            setSelectedText(selectedText.text);
          }
        }, 0);
      }
    });

    // Listen for custom event from visual indicator
    document.addEventListener('chatbotTextSelected', handleCustomTextSelection);

    // Focus the input when modal opens
    if (inputRef.current) {
      setTimeout(() => {
        inputRef.current.focus();
      }, 100);
    }

    // Return cleanup function
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', (e) => {
        if (e.key === 'Shift') {
          setTimeout(() => {
            const selectedText = getTextSelection();
            if (selectedText && selectedText.text.length > 0) {
              setSelectedText(selectedText.text);
            }
          }, 0);
        }
      });
      document.removeEventListener('chatbotTextSelected', handleCustomTextSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async (text) => {
    if (!text.trim() || isLoading || !sessionId) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: text,
      sender: 'user',
      timestamp: new Date(),
      context: selectedText || null
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setIsTyping(true); // Show typing indicator
    setError(null); // Clear any previous errors

    try {
      // Get response from backend with session ID
      const response = await apiService.askQuestion(text, selectedText || '', sessionId);

      // Update session ID if it was generated by the backend
      if (response.session_id && response.session_id !== sessionId) {
        setSessionId(response.session_id);
        localStorage.setItem('chatbot_session_id', response.session_id);
      }

      const botMessage = {
        id: Date.now() + 1,
        text: response.response,
        sender: 'bot',
        timestamp: new Date(),
        sourceChunks: response.source_chunks || [],
        confidence: response.confidence
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I encountered an error. Please try again.",
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
      setError(error.message || "An unknown error occurred");
    } finally {
      setIsLoading(false);
      setIsTyping(false); // Hide typing indicator
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Escape') {
      onClose();
    }
  };

  return (
    <div
      className="chatbot-modal-overlay"
      onClick={onClose}
      onKeyDown={handleKeyDown}
      role="dialog"
      aria-modal="true"
      aria-labelledby="chat-modal-title"
      tabIndex={-1}
    >
      <div
        className="chatbot-modal"
        onClick={e => e.stopPropagation()}
        role="document"
        tabIndex={0}
      >
        <div className="chatbot-header">
          <h3 id="chat-modal-title">Book Assistant</h3>
          <button
            className="chatbot-close-button"
            onClick={onClose}
            aria-label="Close chat"
            aria-describedby="chat-modal-title"
          >
            ×
          </button>
        </div>

        <div
          className="chatbot-messages"
          aria-live="polite"
          aria-relevant="additions"
          role="log"
          aria-label="Chat messages"
        >
          {messages.map((message) => (
            <Message key={message.id} message={message} />
          ))}
          {isTyping && (
            <Message
              message={{
                id: 'typing',
                text: 'Thinking...',
                sender: 'bot',
                timestamp: new Date()
              }}
              isTyping={true}
            />
          )}
          {error && (
            <div className="chatbot-error" role="alert" aria-live="assertive">
              <small>Error: {error}</small>
            </div>
          )}
          <div ref={messagesEndRef} aria-hidden="true" />
        </div>

        {selectedText && (
          <div className="chatbot-selected-text" role="status" aria-live="polite">
            <small>Context: "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</small>
          </div>
        )}

        <InputArea
          onSend={handleSendMessage}
          isLoading={isLoading}
          inputRef={inputRef}
        />
      </div>
    </div>
  );
};

export default ChatModal;