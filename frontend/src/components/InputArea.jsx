import React, { useState } from 'react';

const InputArea = ({ onSend, isLoading, inputRef }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      onSend(inputValue);
      setInputValue('');
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  return (
    <form className="chatbot-input-form" onSubmit={handleSubmit} role="form" aria-label="Chat input form">
      <div className="chatbot-input-container">
        <textarea
          ref={inputRef}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about the book..."
          disabled={isLoading}
          rows="1"
          className="chatbot-textarea"
          aria-label="Type your question here"
          aria-describedby="send-button"
          role="textbox"
          aria-multiline="true"
        />
        <button
          type="submit"
          id="send-button"
          disabled={!inputValue.trim() || isLoading}
          className="chatbot-send-button"
          aria-label="Send message"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </form>
  );
};

export default InputArea;