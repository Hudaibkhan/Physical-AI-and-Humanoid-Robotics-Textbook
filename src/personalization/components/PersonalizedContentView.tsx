import React, { useState, useEffect } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

interface PersonalizedContentViewProps {
  chapterId: string;
  originalContent: string;
  personalizedContent?: string;
  isPersonalized: boolean;
  onTogglePersonalization: () => void;
}

const PersonalizedContentView: React.FC<PersonalizedContentViewProps> = ({
  chapterId,
  originalContent,
  personalizedContent,
  isPersonalized,
  onTogglePersonalization
}) => {
  const [content, setContent] = useState<string>(originalContent);
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    // Update displayed content based on personalization state
    if (isPersonalized && personalizedContent) {
      setContent(personalizedContent);
    } else {
      setContent(originalContent);
    }
  }, [isPersonalized, originalContent, personalizedContent]);

  const handleToggle = () => {
    onTogglePersonalization();
  };

  return (
    <div className="personalized-content-view">
      <div className="content-controls">
        <button
          onClick={handleToggle}
          className={`toggle-btn ${isPersonalized ? 'active' : ''}`}
        >
          {isPersonalized ? 'View Original' : 'View Personalized'}
        </button>

        {isPersonalized && (
          <span className="personalization-indicator">
            🎯 Personalized for your background
          </span>
        )}
      </div>

      <div className="content-display">
        <ReactMarkdown
          remarkPlugins={[remarkGfm]}
          components={{
            // Customize rendering for better display
            code({node, inline, className, children, ...props}) {
              const match = /language-(\w+)/.exec(className || '');
              return !inline && match ? (
                <pre className={className}>
                  <code {...props}>{children}</code>
                </pre>
              ) : (
                <code className={className} {...props}>{children}</code>
              );
            },
            // Preserve citations
            em({node, ...props}) {
              // Check if this is a citation
              const textContent = React.Children.toArray(props.children).join('');
              if (textContent.startsWith('[') && textContent.endsWith(']')) {
                return <em className="citation" {...props} />;
              }
              return <em {...props} />;
            }
          }}
        >
          {content}
        </ReactMarkdown>
      </div>

      {isPersonalized && (
        <div className="personalization-info">
          <small>
            Content adapted based on your profile:
            skill level, hardware background, and learning goals.
          </small>
        </div>
      )}
    </div>
  );
};

export default PersonalizedContentView;