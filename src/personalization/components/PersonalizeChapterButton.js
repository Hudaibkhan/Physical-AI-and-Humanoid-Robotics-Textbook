import React, { useState } from 'react';
import { useAuth } from '../../auth/context/AuthContext';

const PersonalizeChapterButton = ({
  chapterId,
  onPersonalize,
  currentContent
}) => {
  const { state: authState } = useAuth();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const handlePersonalize = async () => {
    if (!authState.isAuthenticated) {
      setError('You must be logged in to personalize content');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Get RAG backend URL from environment variable
      const RAG_BACKEND_URL = process.env.NEXT_PUBLIC_RAG_BACKEND_URL || 'http://localhost:8000';

      const response = await fetch(`${RAG_BACKEND_URL}/personalize-agent`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${authState.token}`
        },
        body: JSON.stringify({
          chapter_content: currentContent,
          user_metadata: {
            skill_level: authState.user?.skill_level,
            hardware_background: authState.user?.hardware_background,
            learning_goal: authState.user?.learning_goal
          }
        })
      });

      if (response.ok) {
        const data = await response.json();
        onPersonalize(chapterId, data.personalized_content);
      } else {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to personalize content');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to personalize content');
    } finally {
      setIsLoading(false);
    }
  };

  if (!authState.isAuthenticated) {
    return (
      <div className="personalize-button-container">
        <button
          onClick={handlePersonalize}
          disabled={true}
          title="Please log in to personalize content"
        >
          Personalize this chapter
        </button>
        <small style={{ color: 'red', marginTop: '0.5rem', display: 'block' }}>
          Please log in to personalize content
        </small>
      </div>
    );
  }

  return (
    <div className="personalize-button-container">
      <button
        onClick={handlePersonalize}
        disabled={isLoading}
        style={{
          backgroundColor: isLoading ? '#ccc' : '#007cba',
          color: 'white',
          border: 'none',
          padding: '10px 15px',
          borderRadius: '4px',
          cursor: isLoading ? 'not-allowed' : 'pointer',
          marginTop: '1rem'
        }}
      >
        {isLoading ? 'Personalizing...' : 'Personalize this chapter'}
      </button>

      {error && (
        <div style={{ color: 'red', marginTop: '0.5rem' }}>
          {error}
        </div>
      )}
    </div>
  );
};

export default PersonalizeChapterButton;