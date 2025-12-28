import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../auth/context/AuthContext';

const PersonalizePage = () => {
  const { state: authState } = useAuth();
  const [isGenerating, setIsGenerating] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState('');
  const [statusMessage, setStatusMessage] = useState('');

  const handleGeneratePersonalized = async () => {
    if (!authState.isAuthenticated) {
      setStatusMessage('Please log in to access personalization features');
      return;
    }

    setIsGenerating(true);
    setStatusMessage('Generating personalized content...');

    // Simulate API call to generate personalized content
    setTimeout(() => {
      const userProfile = authState.user;
      setPersonalizedContent(`
        <h3>Personalized Content Preview</h3>
        <p>Based on your profile:</p>
        <ul>
          <li><strong>Skill Level:</strong> ${userProfile?.skill_level || 'Not specified'}</li>
          <li><strong>Software Background:</strong> ${userProfile?.software_background || 'Not specified'}</li>
          <li><strong>Hardware Background:</strong> ${userProfile?.hardware_background || 'Not specified'}</li>
          <li><strong>Learning Goal:</strong> ${userProfile?.learning_goal || 'Not specified'}</li>
        </ul>
        <p>This is a preview of how the book content would be adapted to your learning style and background.</p>
      `);
      setStatusMessage('Personalized content generated successfully!');
      setIsGenerating(false);
    }, 1500);
  };

  return (
    <Layout title="Personalize Book" description="Generate personalized book content based on your profile">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className="text--center padding-vert--xl">
              <h1>✨ Personalize Your Book Experience</h1>
              <p className="padding-horiz--md">
                This page generates a personalized version of the book content based on your profile and learning preferences.
                The system will adapt explanations, examples, and terminology to match your background and goals.
              </p>
            </div>

            <div className="card margin-vert--lg">
              <div className="card__header">
                <h2>Your Profile</h2>
              </div>
              <div className="card__body">
                {authState.isAuthenticated ? (
                  <div className="profile-grid">
                    <div className="profile-field">
                      <span className="profile-field-label">Email</span>
                      <span className="profile-field-value">{authState.user?.email || 'Not specified'}</span>
                    </div>
                    <div className="profile-field">
                      <span className="profile-field-label">Skill Level</span>
                      <span className={`profile-field-value ${!authState.user?.skill_level ? 'not-set' : ''}`}>
                        {authState.user?.skill_level ? (
                          <span className={`skill-level-badge ${authState.user.skill_level.toLowerCase()}`}>
                            {authState.user.skill_level}
                          </span>
                        ) : 'Not specified'}
                      </span>
                    </div>
                    <div className="profile-field">
                      <span className="profile-field-label">Software Background</span>
                      <span className={`profile-field-value ${!authState.user?.software_background ? 'not-set' : ''}`}>
                        {authState.user?.software_background || 'Not specified'}
                      </span>
                    </div>
                    <div className="profile-field">
                      <span className="profile-field-label">Hardware Background</span>
                      <span className={`profile-field-value ${!authState.user?.hardware_background ? 'not-set' : ''}`}>
                        {authState.user?.hardware_background || 'Not specified'}
                      </span>
                    </div>
                    <div className="profile-field" style={{ gridColumn: '1 / -1' }}>
                      <span className="profile-field-label">Learning Goal</span>
                      <span className={`profile-field-value ${!authState.user?.learning_goal ? 'not-set' : ''}`}>
                        {authState.user?.learning_goal || 'Not specified'}
                      </span>
                    </div>
                  </div>
                ) : (
                  <p><em>Please log in to see your profile information.</em></p>
                )}
              </div>
            </div>

            <div className="margin-vert--lg text--center">
              <button
                className={`button button--primary button--lg ${isGenerating ? 'button--loading' : ''}`}
                onClick={handleGeneratePersonalized}
                disabled={isGenerating || !authState.isAuthenticated}
              >
                {isGenerating ? 'Generating...' : 'Generate Personalized Version'}
              </button>

              {statusMessage && (
                <div className={`margin-vert--md alert ${statusMessage.includes('successfully') ? 'alert--success' : 'alert--info'}`}>
                  {statusMessage}
                </div>
              )}
            </div>

            {personalizedContent && (
              <div className="card margin-vert--lg">
                <div className="card__header">
                  <h2>Personalized Content Preview</h2>
                </div>
                <div className="card__body" dangerouslySetInnerHTML={{ __html: personalizedContent }} />
              </div>
            )}

            <div className="margin-vert--lg">
              <div className="card">
                <div className="card__header">
                  <h3>How Personalization Works</h3>
                </div>
                <div className="card__body">
                  <p>
                    Our personalization system modifies the book's content to better match your learning style and background:
                  </p>
                  <ul>
                    <li><strong>Explanation Depth:</strong> Adjusted based on your experience level</li>
                    <li><strong>Examples:</strong> Tailored to your hardware/software background</li>
                    <li><strong>Terminology:</strong> Adapted to your familiarity level</li>
                    <li><strong>Reading Difficulty:</strong> Matched to your learning goals</li>
                  </ul>
                  <p>
                    This creates a more effective learning experience by presenting concepts in a way that resonates with your background and goals.
                  </p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default PersonalizePage;