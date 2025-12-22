import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../auth/context/AuthContext';

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
  checkboxLabelHover: {
    background: '#f0f0f0',
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
  buttonHover: {
    transform: 'translateY(-1px)',
    boxShadow: '0 4px 12px rgba(102, 126, 234, 0.4)',
  },
  switchButton: {
    background: 'none',
    border: 'none',
    color: '#667eea',
    fontSize: '0.9rem',
    cursor: 'pointer',
    padding: '0.5rem',
    marginTop: '1rem',
    width: '100%',
    textAlign: 'center',
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
  accountCard: {
    background: '#ffffff',
    borderRadius: '12px',
    boxShadow: '0 4px 20px rgba(0, 0, 0, 0.08)',
    overflow: 'hidden',
  },
  accountHeader: {
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    padding: '2rem',
    color: '#fff',
    textAlign: 'center',
  },
  avatar: {
    width: '80px',
    height: '80px',
    borderRadius: '50%',
    background: 'rgba(255,255,255,0.2)',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    margin: '0 auto 1rem',
    fontSize: '2rem',
    fontWeight: 'bold',
  },
  accountBody: {
    padding: '1.5rem 2rem',
  },
  accountRow: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: '0.75rem 0',
    borderBottom: '1px solid #f0f0f0',
  },
  accountLabel: {
    color: '#666',
    fontSize: '0.875rem',
  },
  accountValue: {
    color: '#1a1a2e',
    fontSize: '0.875rem',
    fontWeight: 500,
    textAlign: 'right',
    maxWidth: '60%',
  },
  logoutButton: {
    width: '100%',
    padding: '0.75rem',
    marginTop: '1.5rem',
    background: '#f5f5f5',
    border: '1px solid #ddd',
    borderRadius: '8px',
    color: '#666',
    fontSize: '0.9rem',
    cursor: 'pointer',
    transition: 'background 0.15s',
  },
};

function AuthPage() {
  const { state: authState, login, register, logout } = useAuth();
  const [isLogin, setIsLogin] = useState(true);
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [skillLevel, setSkillLevel] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState([]);
  const [softwareOther, setSoftwareOther] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState([]);
  const [hardwareOther, setHardwareOther] = useState('');
  const [learningGoal, setLearningGoal] = useState('');
  const [error, setError] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [focusedInput, setFocusedInput] = useState(null);

  const handleSoftwareChange = (value) => {
    setSoftwareBackground(prev => {
      if (prev.includes(value)) {
        return prev.filter(v => v !== value);
      }
      // If selecting "none", clear others
      if (value === 'none') {
        return ['none'];
      }
      // If selecting something else, remove "none"
      return [...prev.filter(v => v !== 'none'), value];
    });
  };

  const handleHardwareChange = (value) => {
    setHardwareBackground(prev => {
      if (prev.includes(value)) {
        return prev.filter(v => v !== value);
      }
      if (value === 'none') {
        return ['none'];
      }
      return [...prev.filter(v => v !== 'none'), value];
    });
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
    setError('');
    setIsSubmitting(true);

    try {
      if (isLogin) {
        await login(email, password);
      } else {
        // Format multi-select values for storage
        const softwareValue = formatMultiSelect(softwareBackground, softwareOther, SOFTWARE_OPTIONS);
        const hardwareValue = formatMultiSelect(hardwareBackground, hardwareOther, HARDWARE_OPTIONS);
        const goalLabel = LEARNING_GOALS.find(g => g.value === learningGoal)?.label || learningGoal;

        await register(name, email, password, skillLevel, softwareValue, hardwareValue, goalLabel);

        // Reset form
        setName('');
        setEmail('');
        setPassword('');
        setSkillLevel('');
        setSoftwareBackground([]);
        setSoftwareOther('');
        setHardwareBackground([]);
        setHardwareOther('');
        setLearningGoal('');
      }
    } catch (err) {
      console.error('Auth error:', err);
      setError(err.message || 'An error occurred');
    } finally {
      setIsSubmitting(false);
    }
  };

  // Loading state
  if (authState.loading) {
    return (
      <Layout title="Loading..." description="Checking authentication">
        <div style={styles.pageContainer}>
          <div style={{ textAlign: 'center', color: '#666' }}>
            <div style={{
              width: '40px',
              height: '40px',
              border: '3px solid #f0f0f0',
              borderTopColor: '#667eea',
              borderRadius: '50%',
              animation: 'spin 1s linear infinite',
              margin: '0 auto 1rem'
            }} />
            <p>Loading...</p>
            <style>{`@keyframes spin { to { transform: rotate(360deg); } }`}</style>
          </div>
        </div>
      </Layout>
    );
  }

  // Authenticated - Account page
  if (authState.isAuthenticated) {
    const user = authState.user || {};
    return (
      <Layout title="Account" description="Your account information">
        <div style={styles.pageContainer}>
          <div style={{ ...styles.accountCard, maxWidth: '480px', width: '100%' }}>
            <div style={styles.accountHeader}>
              <div style={styles.avatar}>
                {(user.name?.charAt(0) || user.email?.charAt(0) || 'U').toUpperCase()}
              </div>
              <h2 style={{ margin: 0, fontSize: '1.5rem' }}>{user.name || 'User'}</h2>
              <p style={{ margin: '0.5rem 0 0', opacity: 0.9, fontSize: '0.9rem' }}>{user.email}</p>
            </div>
            <div style={styles.accountBody}>
              <div style={styles.accountRow}>
                <span style={styles.accountLabel}>Skill Level</span>
                <span style={styles.accountValue}>{user.skill_level || '—'}</span>
              </div>
              <div style={styles.accountRow}>
                <span style={styles.accountLabel}>Software Background</span>
                <span style={styles.accountValue}>{user.software_background || '—'}</span>
              </div>
              <div style={styles.accountRow}>
                <span style={styles.accountLabel}>Hardware Background</span>
                <span style={styles.accountValue}>{user.hardware_background || '—'}</span>
              </div>
              <div style={{ ...styles.accountRow, borderBottom: 'none' }}>
                <span style={styles.accountLabel}>Learning Goal</span>
                <span style={styles.accountValue}>{user.learning_goal || '—'}</span>
              </div>
              <button
                style={styles.logoutButton}
                onClick={logout}
                onMouseEnter={(e) => e.target.style.background = '#eee'}
                onMouseLeave={(e) => e.target.style.background = '#f5f5f5'}
              >
                Sign Out
              </button>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  // Login / Register form
  return (
    <Layout title={isLogin ? "Login" : "Register"} description={`Authentication page`}>
      <div style={styles.pageContainer}>
        <div style={styles.card}>
          <div style={styles.cardHeader}>
            <h2 style={styles.cardTitle}>{isLogin ? 'Welcome Back' : 'Create Account'}</h2>
            <p style={styles.cardSubtitle}>
              {isLogin ? 'Sign in to continue learning' : 'Join us to personalize your learning'}
            </p>
          </div>

          <div style={styles.cardBody}>
            {error && <div style={styles.error}>{error}</div>}

            <form onSubmit={handleSubmit}>
              {/* Name - Registration only */}
              {!isLogin && (
                <div style={styles.formGroup}>
                  <label style={styles.label}>
                    Full Name <span style={styles.required}>*</span>
                  </label>
                  <input
                    type="text"
                    style={{
                      ...styles.input,
                      ...(focusedInput === 'name' ? styles.inputFocus : {})
                    }}
                    placeholder="John Doe"
                    value={name}
                    onChange={(e) => setName(e.target.value)}
                    onFocus={() => setFocusedInput('name')}
                    onBlur={() => setFocusedInput(null)}
                    required
                  />
                </div>
              )}

              {/* Email */}
              <div style={styles.formGroup}>
                <label style={styles.label}>
                  Email Address <span style={styles.required}>*</span>
                </label>
                <input
                  type="email"
                  style={{
                    ...styles.input,
                    ...(focusedInput === 'email' ? styles.inputFocus : {})
                  }}
                  placeholder="you@example.com"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  onFocus={() => setFocusedInput('email')}
                  onBlur={() => setFocusedInput(null)}
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
                  style={{
                    ...styles.input,
                    ...(focusedInput === 'password' ? styles.inputFocus : {})
                  }}
                  placeholder="Min 8 characters"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  onFocus={() => setFocusedInput('password')}
                  onBlur={() => setFocusedInput(null)}
                  required
                  minLength={8}
                />
              </div>

              {/* Registration fields */}
              {!isLogin && (
                <>
                  {/* Skill Level - Single Select */}
                  <div style={styles.formGroup}>
                    <label style={styles.label}>Skill Level</label>
                    <select
                      style={styles.select}
                      value={skillLevel}
                      onChange={(e) => setSkillLevel(e.target.value)}
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
                              checked={softwareBackground.includes(opt.value)}
                              onChange={() => handleSoftwareChange(opt.value)}
                            />
                            {opt.label}
                          </label>
                        ))}
                      </div>
                      {softwareBackground.includes('other') && (
                        <input
                          type="text"
                          style={styles.otherInput}
                          placeholder="Please specify..."
                          value={softwareOther}
                          onChange={(e) => setSoftwareOther(e.target.value)}
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
                              checked={hardwareBackground.includes(opt.value)}
                              onChange={() => handleHardwareChange(opt.value)}
                            />
                            {opt.label}
                          </label>
                        ))}
                      </div>
                      {hardwareBackground.includes('other') && (
                        <input
                          type="text"
                          style={styles.otherInput}
                          placeholder="Please specify..."
                          value={hardwareOther}
                          onChange={(e) => setHardwareOther(e.target.value)}
                        />
                      )}
                    </div>
                  </div>

                  {/* Learning Goal - Single Select */}
                  <div style={styles.formGroup}>
                    <label style={styles.label}>Learning Goal</label>
                    <select
                      style={styles.select}
                      value={learningGoal}
                      onChange={(e) => setLearningGoal(e.target.value)}
                    >
                      <option value="">What's your main goal?</option>
                      {LEARNING_GOALS.map(opt => (
                        <option key={opt.value} value={opt.value}>{opt.label}</option>
                      ))}
                    </select>
                  </div>
                </>
              )}

              {/* Submit Button */}
              <button
                type="submit"
                style={{
                  ...styles.button,
                  ...(isSubmitting ? styles.buttonDisabled : {})
                }}
                disabled={isSubmitting}
                onMouseEnter={(e) => !isSubmitting && (e.target.style.transform = 'translateY(-1px)')}
                onMouseLeave={(e) => e.target.style.transform = 'none'}
              >
                {isSubmitting ? (
                  <span>Processing...</span>
                ) : (
                  isLogin ? 'Sign In' : 'Create Account'
                )}
              </button>
            </form>

            {/* Toggle Login/Register */}
            <button
              style={styles.switchButton}
              onClick={() => {
                setIsLogin(!isLogin);
                setError('');
              }}
            >
              {isLogin ? "Don't have an account? Sign up" : "Already have an account? Sign in"}
            </button>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default AuthPage;
