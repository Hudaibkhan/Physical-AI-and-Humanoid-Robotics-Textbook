import React from 'react';

const Message = ({ message, isTyping = false }) => {
  const isUser = message.sender === 'user';

  // Calculate confidence level for visual indicator
  const getConfidenceLevel = (confidence) => {
    if (confidence === undefined || confidence === null) return null;
    if (confidence >= 0.7) return 'high';
    if (confidence >= 0.4) return 'medium';
    return 'low';
  };

  const confidenceLevel = getConfidenceLevel(message.confidence);

  return (
    <div
      className={`chatbot-message ${isUser ? 'user-message' : 'bot-message'}`}
      role="logitem"
      aria-label={isUser ? "User message" : "Assistant response"}
    >
      <div className="chatbot-message-content">
        {isTyping ? (
          <div className="typing-indicator" aria-label="Assistant is typing" role="status" aria-live="polite">
            <div className="typing-dot" aria-hidden="true"></div>
            <div className="typing-dot" aria-hidden="true"></div>
            <div className="typing-dot" aria-hidden="true"></div>
          </div>
        ) : (
          <>
            <div className="message-text">
              {message.text}
            </div>

            {/* Confidence indicator for bot messages */}
            {!isUser && confidenceLevel && (
              <div
                className={`confidence-indicator confidence-${confidenceLevel}`}
                role="status"
                aria-live="polite"
              >
                <small>
                  Confidence: {confidenceLevel}
                  {message.confidence !== undefined && ` (${(message.confidence * 100).toFixed(1)}%)`}
                </small>
              </div>
            )}
          </>
        )}

        {/* Source chunks for bot messages */}
        {message.sourceChunks && message.sourceChunks.length > 0 && !isUser && (
          <details className="source-details" role="group" aria-label="Supporting sources">
            <summary aria-label={`Show ${message.sourceChunks.length} supporting sources`}>
              Sources ({message.sourceChunks.length})
            </summary>
            <ul className="source-list" role="list">
              {message.sourceChunks.map((chunk, index) => (
                <li key={index} className="source-item" role="listitem">
                  <small>"{chunk.content.substring(0, 150)}{chunk.content.length > 150 ? '...' : ''}"</small>
                  <div className="source-similarity" role="status" aria-live="polite">
                    Similarity: {(chunk.similarity_score * 100).toFixed(1)}%
                  </div>
                </li>
              ))}
            </ul>
          </details>
        )}
      </div>
    </div>
  );
};

export default Message;