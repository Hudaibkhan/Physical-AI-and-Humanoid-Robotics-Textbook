import React, { useState } from 'react';
import { useAuth } from '../../auth/context/AuthContext';

interface PersonalizeChapterButtonProps {
  chapterId: string;
  onPersonalize: (chapterId: string, content: string) => void;
  currentContent: string;
}

const PersonalizeChapterButton: React.FC<PersonalizeChapterButtonProps> = ({
  chapterId,
  onPersonalize,
  currentContent
}) => {
  const { state: authState } = useAuth();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handlePersonalize = async () => {
    if (!authState.isAuthenticated) {
      setError('You must be logged in to personalize content');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Call the personalization API
      const response = await fetch('/api/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          currentContent: currentContent,
          user_metadata: {
            software_background: authState.user?.software_background,
            hardware_background: authState.user?.hardware_background,
            learning_goal: authState.user?.learning_goal
          }
        })
      });

      if (response.ok) {
        const data = await response.json();
        onPersonalize(chapterId, data.personalized_content);
      } else {
        throw new Error('Failed to personalize content');
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
          cursor: isLoading ? 'not-allowed' : 'pointer'
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