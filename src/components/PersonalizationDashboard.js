import React from 'react';
import { useAuth } from '../auth/context/AuthContext';

const PersonalizationDashboard = ({ isVisible }) => {
  const { state: authState } = useAuth();

  if (!isVisible || !authState.isAuthenticated) {
    return null;
  }

  // Sample recommended chapters based on user profile
  const getRecommendedChapters = () => {
    const recommendations = [];
    const user = authState.user;

    if (user?.learning_goal?.toLowerCase().includes('ai')) {
      recommendations.push(
        { id: 'ch-ai-fundamentals', title: 'AI Fundamentals', link: '/docs/module3-ai-brain-isaac/chapter1' },
        { id: 'ch-machine-learning', title: 'Machine Learning Basics', link: '/docs/module3-ai-brain-isaac/chapter2' }
      );
    }

    if (user?.learning_goal?.toLowerCase().includes('robotics')) {
      recommendations.push(
        { id: 'ch-robotics-basics', title: 'Robotics Fundamentals', link: '/docs/module1-ros2-nervous-system/chapter1' },
        { id: 'ch-navigation', title: 'Navigation and Control', link: '/docs/module1-ros2-nervous-system/chapter3' }
      );
    }

    if (user?.software_background?.includes('beginner')) {
      recommendations.push(
        { id: 'ch-getting-started', title: 'Getting Started with ROS2', link: '/docs/module1-ros2-nervous-system/chapter0' }
      );
    }

    if (recommendations.length === 0) {
      // Default recommendations
      recommendations.push(
        { id: 'ch-intro', title: 'Introduction to Physical AI', link: '/' },
        { id: 'ch-overview', title: 'Course Overview', link: '/docs/weekly-roadmap/' }
      );
    }

    return recommendations;
  };

  const recommendedChapters = getRecommendedChapters();

  return (
    <div className="personalization-dashboard" style={{
      backgroundColor: '#f8fafc',
      border: '1px solid #e2e8f0',
      borderRadius: '8px',
      padding: '1.5rem',
      marginBottom: '1.5rem'
    }}>
      <h3 style={{ marginBottom: '1rem', color: '#0f172a' }}>Personalized Recommendations</h3>

      <div style={{ marginBottom: '1rem' }}>
        <p style={{ marginBottom: '0.5rem', fontWeight: '500' }}>Based on your profile:</p>
        <ul style={{ marginBottom: '1rem', paddingLeft: '1.5rem' }}>
          {authState.user?.software_background && (
            <li>Software Background: {authState.user.software_background}</li>
          )}
          {authState.user?.hardware_background && (
            <li>Hardware Background: {authState.user.hardware_background}</li>
          )}
          {authState.user?.learning_goal && (
            <li>Learning Goal: {authState.user.learning_goal}</li>
          )}
        </ul>
      </div>

      <div>
        <p style={{ marginBottom: '0.5rem', fontWeight: '500' }}>Recommended for you:</p>
        <ul style={{ paddingLeft: '1.5rem' }}>
          {recommendedChapters.map((chapter) => (
            <li key={chapter.id} style={{ marginBottom: '0.25rem' }}>
              <a href={chapter.link} style={{ color: '#007cba', textDecoration: 'underline' }}>
                {chapter.title}
              </a>
            </li>
          ))}
        </ul>
      </div>
    </div>
  );
};

export default PersonalizationDashboard;