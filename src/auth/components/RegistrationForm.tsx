import React, { useState } from 'react';

interface RegistrationFormData {
  name: string;
  email: string;
  password: string;
  skill_level: string;
  software_background: string;
  hardware_background: string;
  learning_goal: string;
}

interface RegistrationFormProps {
  onRegister: (data: RegistrationFormData) => Promise<void>;
  onCancel?: () => void;
}

const RegistrationForm: React.FC<RegistrationFormProps> = ({ onRegister, onCancel }) => {
  const [formData, setFormData] = useState<RegistrationFormData>({
    name: '',
    email: '',
    password: '',
    skill_level: '',
    software_background: '',
    hardware_background: '',
    learning_goal: '',
  });

  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      await onRegister(formData);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Registration failed');
    } finally {
      setLoading(false);
    }
  };

  // Skill level options
  const skillLevelOptions = [
    { value: '', label: 'Select your skill level' },
    { value: 'beginner', label: 'Beginner' },
    { value: 'intermediate', label: 'Intermediate' },
    { value: 'advanced', label: 'Advanced' },
    { value: 'expert', label: 'Expert' },
  ];

  return (
    <div className="registration-form">
      <h2>Create Account</h2>

      {error && (
        <div className="error-message" style={{ color: 'red', marginBottom: '1rem' }}>
          {error}
        </div>
      )}

      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="name">Name: *</label>
          <input
            type="text"
            id="name"
            name="name"
            value={formData.name}
            onChange={handleChange}
            placeholder="Enter your full name"
            required
            disabled={loading}
          />
        </div>

        <div className="form-group">
          <label htmlFor="email">Email: *</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            placeholder="Enter your email"
            required
            disabled={loading}
          />
        </div>

        <div className="form-group">
          <label htmlFor="password">Password: *</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            placeholder="Create a password (min 8 characters)"
            required
            minLength={8}
            disabled={loading}
          />
        </div>

        <div className="form-group">
          <label htmlFor="skill_level">Skill Level:</label>
          <select
            id="skill_level"
            name="skill_level"
            value={formData.skill_level}
            onChange={handleChange}
            disabled={loading}
          >
            {skillLevelOptions.map(option => (
              <option key={option.value} value={option.value}>
                {option.label}
              </option>
            ))}
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="software_background">Software Background:</label>
          <textarea
            id="software_background"
            name="software_background"
            value={formData.software_background}
            onChange={handleChange}
            placeholder="Describe your software experience (e.g., Python, JavaScript, ROS, ML frameworks)"
            rows={3}
            maxLength={500}
            disabled={loading}
          />
        </div>

        <div className="form-group">
          <label htmlFor="hardware_background">Hardware Background:</label>
          <textarea
            id="hardware_background"
            name="hardware_background"
            value={formData.hardware_background}
            onChange={handleChange}
            placeholder="Describe your hardware experience (e.g., Arduino, Raspberry Pi, robotics kits, GPU setup)"
            rows={3}
            maxLength={500}
            disabled={loading}
          />
        </div>

        <div className="form-group">
          <label htmlFor="learning_goal">Learning Goal:</label>
          <textarea
            id="learning_goal"
            name="learning_goal"
            value={formData.learning_goal}
            onChange={handleChange}
            placeholder="What do you want to learn or build? (e.g., Build a humanoid robot, Master ROS2, Implement vision AI)"
            rows={3}
            maxLength={500}
            disabled={loading}
          />
        </div>

        <div className="form-actions">
          {onCancel && (
            <button type="button" onClick={onCancel} disabled={loading}>
              Cancel
            </button>
          )}
          <button type="submit" disabled={loading}>
            {loading ? 'Registering...' : 'Register'}
          </button>
        </div>
      </form>
    </div>
  );
};

export default RegistrationForm;