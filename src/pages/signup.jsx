import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useAuth } from '../auth/context/AuthContext';
import { useHistory } from '@docusaurus/router';

// Dropdown options
const SKILL_LEVELS = [
  { value: 'beginner', label: 'Beginner' },
  { value: 'intermediate', label: 'Intermediate' },
  { value: 'advanced', label: 'Advanced' },
];

const SOFTWARE_OPTIONS = [
  { value: 'none', label: 'None' },
  { value: 'python', label: 'Python' },
  { value: 'javascript', label: 'JavaScript / TypeScript' },
  { value: 'react', label: 'React / Next.js' },
  { value: 'ml', label: 'Machine Learning' },
  { value: 'ros', label: 'ROS (Robot Operating System)' },
  { value: 'cpp', label: 'C / C++' },
  { value: 'other', label: 'Other' },
];

const HARDWARE_OPTIONS = [
  { value: 'none', label: 'None' },
  { value: 'arduino', label: 'Arduino' },
  { value: 'raspberry-pi', label: 'Raspberry Pi' },
  { value: 'robotics-kits', label: 'Robotics Kits' },
  { value: 'sensors-iot', label: 'Sensors / IoT' },
  { value: 'gpu-cuda', label: 'GPU / CUDA Setup' },
  { value: 'other', label: 'Other' },
];

const LEARNING_GOALS = [
  { value: 'learn-basics', label: 'Learn Basics' },
  { value: 'build-projects', label: 'Build Projects' },
  { value: 'prepare-job', label: 'Prepare for Job' },
  { value: 'research-ai', label: 'Research / AI Development' },
  { value: 'robotics-physical-ai', label: 'Robotics / Physical AI' },
];

// Styles
const styles = {
  pageContainer: {
    minHeight: '80vh',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    padding: '2rem 1rem',
    background: 'linear-gradient(135deg, #f5f7fa 0%, #e4e8ec 100%)',
  },
  card: {
    background: '#ffffff',
    borderRadius: '12px',
    boxShadow: '0 4px 20px rgba(0, 0, 0, 0.08)',
    width: '100%',
    maxWidth: '480px',
    overflow: 'hidden',
  },
  cardHeader: {
    padding: '2rem 2rem 1rem',
    textAlign: 'center',
    borderBottom: '1px solid #eee',
  },
  cardTitle: {
    margin: 0,
    fontSize: '1.75rem',
    fontWeight: 600,
    color: '#1a1a2e',
  },
  cardSubtitle: {
    margin: '0.5rem 0 0',
    fontSize: '0.9rem',
    color: '#666',
  },
  cardBody: {
    padding: '1.5rem 2rem 2rem',
  },
  formGroup: {
    marginBottom: '1.25rem',
  },
  label: {
    display: 'block',
    marginBottom: '0.5rem',
    fontSize: '0.875rem',
    fontWeight: 500,
    color: '#333',
  },
  required: {
    color: '#e53e3e',
    marginLeft: '2px',
  },
  input: {
    width: '100%',
    padding: '0.75rem 1rem',
    fontSize: '0.95rem',
    border: '1px solid #ddd',
    borderRadius: '8px',
    transition: 'border-color 0.2s, box-shadow 0.2s',
    outline: 'none',
    boxSizing: 'border-box',
  },
  inputFocus: {
    borderColor: '#667eea',
    boxShadow: '0 0 0 3px rgba(102, 126, 234, 0.1)',
  },
  select: {
    width: '100%',
    padding: '0.75rem 1rem',
    fontSize: '0.95rem',
    border: '1px solid #ddd',
    borderRadius: '8px',
    background: '#fff',
    cursor: 'pointer',
    outline: 'none',
    boxSizing: 'border-box',
  },
  multiSelectContainer: {
    border: '1px solid #ddd',
    borderRadius: '8px',
    padding: '0.5rem',
    background: '#fafafa',
  },
  checkboxGroup: {
    display: 'grid',
    gridTemplateColumns: 'repeat(2, 1fr)',
    gap: '0.5rem',
  },
  checkboxLabel: {
    display: 'flex',
    alignItems: 'center',
    padding: '0.5rem 0.75rem',
    fontSize: '0.85rem',
    borderRadius: '6px',
    cursor: 'pointer',
    transition: 'background 0.15s',
    color: '#444',
  },
  checkbox: {
    marginRight: '0.5rem',
    width: '16px',
    height: '16px',
    cursor: 'pointer',
  },
  otherInput: {
    marginTop: '0.5rem',
    padding: '0.5rem 0.75rem',
    fontSize: '0.85rem',
    border: '1px solid #ddd',
    borderRadius: '6px',
    width: '100%',
    boxSizing: 'border-box',
  },
  helperText: {
    fontSize: '0.75rem',
    color: '#888',
    marginTop: '0.25rem',
  },
  button: {
    width: '100%',
    padding: '0.875rem 1.5rem',
    fontSize: '1rem',
    fontWeight: 600,
    color: '#fff',
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
    transition: 'transform 0.15s, box-shadow 0.15s',
  },
  buttonDisabled: {
    opacity: 0.7,
    cursor: 'not-allowed',
  },
  error: {
    background: '#fee2e2',
    border: '1px solid #fecaca',
    color: '#dc2626',
    padding: '0.75rem 1rem',
    borderRadius: '8px',
    fontSize: '0.875rem',
    marginBottom: '1rem',
  },
};

const SignupPage = () => {
  const { register } = useAuth();
  const history = useHistory();

  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    skill_level: '',
    software_background: [],
    hardware_background: [],
    learning_goal: ''
  });

  const [softwareOther, setSoftwareOther] = useState('');
  const [hardwareOther, setHardwareOther] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const handleSoftwareChange = (value) => {
    setFormData(prev => ({
      ...prev,
      software_background: prev.software_background.includes(value)
        ? prev.software_background.filter(v => v !== value)
        : value === 'none'
          ? ['none']
          : [...prev.software_background.filter(v => v !== 'none'), value]
    }));
  };

  const handleHardwareChange = (value) => {
    setFormData(prev => ({
      ...prev,
      hardware_background: prev.hardware_background.includes(value)
        ? prev.hardware_background.filter(v => v !== value)
        : value === 'none'
          ? ['none']
          : [...prev.hardware_background.filter(v => v !== 'none'), value]
    }));
  };

  const formatMultiSelect = (selected, otherText, options) => {
    if (selected.length === 0) return '';

    const labels = selected.map(val => {
      if (val === 'other' && otherText) {
        return `Other: ${otherText}`;
      }
      const opt = options.find(o => o.value === val);
      return opt ? opt.label : val;
    });

    return labels.join(', ');
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      console.log('Attempting registration with data:', {
        name: formData.name,
        email: formData.email,
        skill_level: formData.skill_level,
        software_background: formData.software_background,
        hardware_background: formData.hardware_background,
        learning_goal: formData.learning_goal
      });

      // Format multi-select values for storage
      const softwareValue = formatMultiSelect(formData.software_background, softwareOther, SOFTWARE_OPTIONS);
      const hardwareValue = formatMultiSelect(formData.hardware_background, hardwareOther, HARDWARE_OPTIONS);
      const goalLabel = LEARNING_GOALS.find(g => g.value === formData.learning_goal)?.label || formData.learning_goal;

      await register(
        formData.name,
        formData.email,
        formData.password,
        formData.skill_level,
        softwareValue,
        hardwareValue,
        goalLabel
      );

      console.log('Registration successful!');
      // Redirect to home page or personalization settings on success
      history.push('/');
    } catch (err) {
      console.error('Registration error:', err);
      setError(err.message || 'Registration failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleChange = (field, value) => {
    setFormData(prev => ({ ...prev, [field]: value }));
  };

  return (
    <Layout title="Sign Up" description="Create a new account">
      <div style={styles.pageContainer}>
        <div style={styles.card}>
          <div style={styles.cardHeader}>
            <h2 style={styles.cardTitle}>Create Account</h2>
            <p style={styles.cardSubtitle}>Join us to personalize your learning</p>
          </div>

          <div style={styles.cardBody}>
            {error && <div style={styles.error}>{error}</div>}

            <form onSubmit={handleSubmit}>
              {/* Name */}
              <div style={styles.formGroup}>
                <label style={styles.label}>
                  Full Name <span style={styles.required}>*</span>
                </label>
                <input
                  type="text"
                  style={styles.input}
                  placeholder="John Doe"
                  value={formData.name}
                  onChange={(e) => handleChange('name', e.target.value)}
                  disabled={loading}
                  required
                />
              </div>

              {/* Email */}
              <div style={styles.formGroup}>
                <label style={styles.label}>
                  Email Address <span style={styles.required}>*</span>
                </label>
                <input
                  type="email"
                  style={styles.input}
                  placeholder="you@example.com"
                  value={formData.email}
                  onChange={(e) => handleChange('email', e.target.value)}
                  disabled={loading}
                  required
                />
              </div>

              {/* Password */}
              <div style={styles.formGroup}>
                <label style={styles.label}>
                  Password <span style={styles.required}>*</span>
                </label>
                <input
                  type="password"
                  style={styles.input}
                  placeholder="Min 8 characters"
                  value={formData.password}
                  onChange={(e) => handleChange('password', e.target.value)}
                  disabled={loading}
                  required
                  minLength="8"
                />
              </div>

              {/* Skill Level - Single Select */}
              <div style={styles.formGroup}>
                <label style={styles.label}>Skill Level</label>
                <select
                  style={styles.select}
                  value={formData.skill_level}
                  onChange={(e) => handleChange('skill_level', e.target.value)}
                  disabled={loading}
                >
                  <option value="">Select your level</option>
                  {SKILL_LEVELS.map(opt => (
                    <option key={opt.value} value={opt.value}>{opt.label}</option>
                  ))}
                </select>
              </div>

              {/* Software Background - Multi Select */}
              <div style={styles.formGroup}>
                <label style={styles.label}>Software Background</label>
                <p style={styles.helperText}>Select all that apply</p>
                <div style={styles.multiSelectContainer}>
                  <div style={styles.checkboxGroup}>
                    {SOFTWARE_OPTIONS.map(opt => (
                      <label
                        key={opt.value}
                        style={styles.checkboxLabel}
                        onMouseEnter={(e) => e.currentTarget.style.background = '#f0f0f0'}
                        onMouseLeave={(e) => e.currentTarget.style.background = 'transparent'}
                      >
                        <input
                          type="checkbox"
                          style={styles.checkbox}
                          checked={formData.software_background.includes(opt.value)}
                          onChange={() => handleSoftwareChange(opt.value)}
                          disabled={loading}
                        />
                        {opt.label}
                      </label>
                    ))}
                  </div>
                  {formData.software_background.includes('other') && (
                    <input
                      type="text"
                      style={styles.otherInput}
                      placeholder="Please specify..."
                      value={softwareOther}
                      onChange={(e) => setSoftwareOther(e.target.value)}
                      disabled={loading}
                    />
                  )}
                </div>
              </div>

              {/* Hardware Background - Multi Select */}
              <div style={styles.formGroup}>
                <label style={styles.label}>Hardware Background</label>
                <p style={styles.helperText}>Select all that apply</p>
                <div style={styles.multiSelectContainer}>
                  <div style={styles.checkboxGroup}>
                    {HARDWARE_OPTIONS.map(opt => (
                      <label
                        key={opt.value}
                        style={styles.checkboxLabel}
                        onMouseEnter={(e) => e.currentTarget.style.background = '#f0f0f0'}
                        onMouseLeave={(e) => e.currentTarget.style.background = 'transparent'}
                      >
                        <input
                          type="checkbox"
                          style={styles.checkbox}
                          checked={formData.hardware_background.includes(opt.value)}
                          onChange={() => handleHardwareChange(opt.value)}
                          disabled={loading}
                        />
                        {opt.label}
                      </label>
                    ))}
                  </div>
                  {formData.hardware_background.includes('other') && (
                    <input
                      type="text"
                      style={styles.otherInput}
                      placeholder="Please specify..."
                      value={hardwareOther}
                      onChange={(e) => setHardwareOther(e.target.value)}
                      disabled={loading}
                    />
                  )}
                </div>
              </div>

              {/* Learning Goal - Single Select */}
              <div style={styles.formGroup}>
                <label style={styles.label}>Learning Goal</label>
                <select
                  style={styles.select}
                  value={formData.learning_goal}
                  onChange={(e) => handleChange('learning_goal', e.target.value)}
                  disabled={loading}
                >
                  <option value="">What's your main goal?</option>
                  {LEARNING_GOALS.map(opt => (
                    <option key={opt.value} value={opt.value}>{opt.label}</option>
                  ))}
                </select>
              </div>

              {/* Submit Button */}
              <div style={{ marginBottom: '1rem' }}>
                <button
                  type="submit"
                  style={{
                    ...styles.button,
                    ...(loading ? styles.buttonDisabled : {})
                  }}
                  disabled={loading}
                >
                  {loading ? 'Creating Account...' : 'Sign Up'}
                </button>
              </div>
            </form>

            <div style={{ textAlign: 'center' }}>
              <p>
                Already have an account?{' '}
                <Link to="/login" style={{ color: '#667eea', fontWeight: '500' }}>
                  Log in here
                </Link>
              </p>
              <br />
              <Link
                to="/auth"
                style={{
                  color: '#666',
                  fontSize: '0.9rem',
                  textDecoration: 'none'
                }}
              >
                Back to Main Auth
              </Link>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default SignupPage;