import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../auth/context/AuthContext';

function PersonalizationSettingsPage() {
  const { state: authState, register } = useAuth();
  const [softwareBackground, setSoftwareBackground] = useState(authState.user?.software_background || '');
  const [hardwareBackground, setHardwareBackground] = useState(authState.user?.hardware_background || '');
  const [learningGoal, setLearningGoal] = useState(authState.user?.learning_goal || '');
  const [isEditing, setIsEditing] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [message, setMessage] = useState('');

  useEffect(() => {
    if (authState.user) {
      setSoftwareBackground(authState.user.software_background || '');
      setHardwareBackground(authState.user.hardware_background || '');
      setLearningGoal(authState.user.learning_goal || '');
    }
  }, [authState.user]);

  const handleSave = async (e) => {
    e.preventDefault();
    setIsSaving(true);
    setMessage('');

    try {
      // Call the profile update API
      const response = await fetch('/api/auth/profile', {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          software_background,
          hardware_background,
          learning_goal
        })
      });

      if (response.ok) {
        const data = await response.json();
        setMessage(data.message || 'Profile updated successfully!');
        setIsEditing(false);
      } else {
        throw new Error('Failed to update profile');
      }
    } catch (error) {
      setMessage('Error updating profile: ' + error.message);
    } finally {
      setIsSaving(false);
    }
  };

  // CRITICAL: Check loading BEFORE checking isAuthenticated
  // This prevents false "not logged in" state while session is being restored
  if (authState.loading) {
    return (
      <Layout title="Personalization Settings">
        <div className="container margin-vert--lg text--center">
          <p>Loading...</p>
        </div>
      </Layout>
    );
  }

  if (!authState.isAuthenticated) {
    return (
      <Layout title="Personalization Settings" description="Please log in to access personalization settings">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <div className="text--center padding-vert--xl">
                <h1>Personalization Settings</h1>
                <p>Please log in to access personalization settings.</p>
                <a href="/auth" className="button button--primary">
                  Login
                </a>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Personalization Settings" description="Manage your personalization preferences">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className="card">
              <div className="card__header">
                <h2>Personalization Settings</h2>
              </div>
              <div className="card__body">
                <p>Customize your learning experience based on your background and goals.</p>

                <form onSubmit={handleSave}>
                  <div className="form-group margin-bottom--lg">
                    <label htmlFor="softwareBackground"><strong>Software Background:</strong></label>
                    <select
                      id="softwareBackground"
                      className="form-control"
                      value={softwareBackground}
                      onChange={(e) => setSoftwareBackground(e.target.value)}
                      disabled={!isEditing}
                    >
                      <option value="web_dev">Web Development</option>
                      <option value="backend">Backend Development</option>
                      <option value="ai_ml">AI/ML</option>
                      <option value="beginner">Beginner</option>
                      <option value="intermediate">Intermediate</option>
                      <option value="advanced">Advanced</option>
                    </select>
                  </div>

                  <div className="form-group margin-bottom--lg">
                    <label htmlFor="hardwareBackground"><strong>Hardware Background:</strong></label>
                    <input
                      type="text"
                      id="hardwareBackground"
                      className="form-control"
                      placeholder="e.g., Laptop, PC, Robotics Kit, GPU, etc."
                      value={hardwareBackground}
                      onChange={(e) => setHardwareBackground(e.target.value)}
                      disabled={!isEditing}
                    />
                  </div>

                  <div className="form-group margin-bottom--lg">
                    <label htmlFor="learningGoal"><strong>Learning Goal:</strong></label>
                    <input
                      type="text"
                      id="learningGoal"
                      className="form-control"
                      placeholder="What do you want to learn? (AI, Robotics, Full Stack, etc.)"
                      value={learningGoal}
                      onChange={(e) => setLearningGoal(e.target.value)}
                      disabled={!isEditing}
                    />
                  </div>

                  {message && (
                    <div className={`alert ${message.includes('Error') ? 'alert--danger' : 'alert--success'} margin-bottom--md`}>
                      {message}
                    </div>
                  )}

                  <div className="button-group">
                    {!isEditing ? (
                      <button
                        type="button"
                        className="button button--primary"
                        onClick={() => setIsEditing(true)}
                      >
                        Edit Profile
                      </button>
                    ) : (
                      <>
                        <button
                          type="submit"
                          className="button button--primary"
                          disabled={isSaving}
                        >
                          {isSaving ? 'Saving...' : 'Save Changes'}
                        </button>
                        <button
                          type="button"
                          className="button button--secondary"
                          onClick={() => {
                            setIsEditing(false);
                            // Reset to original values
                            setSoftwareBackground(authState.user.software_background || '');
                            setHardwareBackground(authState.user.hardware_background || '');
                            setLearningGoal(authState.user.learning_goal || '');
                            setMessage('');
                          }}
                          disabled={isSaving}
                        >
                          Cancel
                        </button>
                      </>
                    )}
                  </div>
                </form>
              </div>
            </div>

            <div className="margin-vert--lg">
              <div className="card">
                <div className="card__header">
                  <h3>How Personalization Works</h3>
                </div>
                <div className="card__body">
                  <p>
                    Based on your profile information, we tailor the content to match your background and learning goals.
                    This helps you focus on the most relevant information and learn more effectively.
                  </p>
                  <ul>
                    <li>Content examples are adapted to your hardware/software background</li>
                    <li>Complexity level is adjusted based on your experience</li>
                    <li>Learning paths are customized to help you achieve your goals</li>
                  </ul>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default PersonalizationSettingsPage;