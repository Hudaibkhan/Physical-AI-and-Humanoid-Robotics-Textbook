import React, { useState, useEffect } from 'react';
import { useAuth } from '../auth/context/AuthContext';

const PersonalizeButton = ({ chapterId, originalContent, onPersonalize }) => {
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
      // In a real implementation, this would call the personalization API
      // For now, simulate API call
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Mock personalized content
      const mockPersonalizedContent = `**PERSONALIZED CONTENT**\n\n${originalContent}\n\n*This content has been tailored to your background: ${authState.user?.skill_level || 'beginner'} skill level, with focus on ${authState.user?.hardware_background || 'general robotics'} concepts.*`;

      onPersonalize(chapterId, mockPersonalizedContent);
    } catch (err) {
      setError(err.message || 'Failed to personalize content');
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
          style={{
            backgroundColor: '#ccc',
            color: '#666',
            border: '1px solid #ccc',
            padding: '10px 15px',
            borderRadius: '4px',
            cursor: 'not-allowed',
            marginTop: '1rem'
          }}
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

export default PersonalizeButton;