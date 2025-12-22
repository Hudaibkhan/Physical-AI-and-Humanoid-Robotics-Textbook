import React, { useState } from 'react';
import { useAuth } from '../auth/context/AuthContext';

const NavbarAuth = () => {
  const { state: authState, login, register, logout } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);
  const [isLogin, setIsLogin] = useState(true); // true for login, false for register
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const [learningGoal, setLearningGoal] = useState('');

  const handleLogin = async (e) => {
    e.preventDefault();
    try {
      await login(email, password);
      setShowAuthModal(false);
      setEmail('');
      setPassword('');
    } catch (error) {
      alert('Login failed: ' + error.message);
    }
  };

  const handleRegister = async (e) => {
    e.preventDefault();
    try {
      await register(email, password, softwareBackground, hardwareBackground, learningGoal);
      setShowAuthModal(false);
      setEmail('');
      setPassword('');
      setSoftwareBackground('');
      setHardwareBackground('');
      setLearningGoal('');
    } catch (error) {
      alert('Registration failed: ' + error.message);
    }
  };

  const handleLogout = () => {
    logout();
  };

  if (authState.loading) {
    return (
      <div className="navbar__item">
        <div style={{
          width: '80px',
          height: '32px',
          background: 'linear-gradient(90deg, #e0e0e0 25%, #f0f0f0 50%, #e0e0e0 75%)',
          backgroundSize: '200% 100%',
          borderRadius: '4px',
          animation: 'shimmer 1.5s infinite'
        }} />
        <style>{`
          @keyframes shimmer {
            0% { background-position: 200% 0; }
            100% { background-position: -200% 0; }
          }
        `}</style>
      </div>
    );
  }

  return (
    <>
      {authState.isAuthenticated ? (
        <div className="navbar__item" style={{ position: 'relative' }}>
          <button
            className="navbar__link"
            onClick={() => setShowDropdown(!showDropdown)}
            onBlur={() => setTimeout(() => setShowDropdown(false), 200)}
            style={{
              display: 'flex',
              alignItems: 'center',
              gap: '0.5rem',
              background: 'none',
              border: 'none',
              cursor: 'pointer',
              padding: '0.5rem',
              fontSize: 'inherit'
            }}
          >
            <div style={{
              width: '28px',
              height: '28px',
              borderRadius: '50%',
              background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              color: 'white',
              fontSize: '0.85rem',
              fontWeight: 'bold'
            }}>
              {(authState.user?.name?.charAt(0) || authState.user?.email?.charAt(0) || 'U').toUpperCase()}
            </div>
            {authState.user?.name || authState.user?.email?.split('@')[0] || 'User'} ▾
          </button>

          {showDropdown && (
            <div style={{
              position: 'absolute',
              top: '100%',
              right: 0,
              backgroundColor: 'var(--ifm-dropdown-background-color, white)',
              border: '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '4px',
              boxShadow: '0 2px 8px rgba(0,0,0,0.15)',
              minWidth: '150px',
              zIndex: 1000,
              marginTop: '0.25rem'
            }}>
              <a
                href="/auth"
                className="dropdown__link"
                style={{
                  display: 'block',
                  padding: '0.5rem 1rem',
                  textDecoration: 'none',
                  color: 'var(--ifm-font-color-base)',
                  whiteSpace: 'nowrap'
                }}
                onMouseEnter={(e) => e.target.style.backgroundColor = 'var(--ifm-color-emphasis-100)'}
                onMouseLeave={(e) => e.target.style.backgroundColor = 'transparent'}
              >
                Account
              </a>
              <a
                href="/personalization-settings"
                className="dropdown__link"
                style={{
                  display: 'block',
                  padding: '0.5rem 1rem',
                  textDecoration: 'none',
                  color: 'var(--ifm-font-color-base)',
                  whiteSpace: 'nowrap'
                }}
                onMouseEnter={(e) => e.target.style.backgroundColor = 'var(--ifm-color-emphasis-100)'}
                onMouseLeave={(e) => e.target.style.backgroundColor = 'transparent'}
              >
                Personalization
              </a>
              <a
                href="#"
                className="dropdown__link"
                onClick={(e) => {
                  e.preventDefault();
                  setShowDropdown(false);
                  handleLogout();
                }}
                style={{
                  display: 'block',
                  padding: '0.5rem 1rem',
                  textDecoration: 'none',
                  color: 'var(--ifm-font-color-base)',
                  whiteSpace: 'nowrap',
                  borderTop: '1px solid var(--ifm-color-emphasis-200)'
                }}
                onMouseEnter={(e) => e.target.style.backgroundColor = 'var(--ifm-color-emphasis-100)'}
                onMouseLeave={(e) => e.target.style.backgroundColor = 'transparent'}
              >
                Logout
              </a>
            </div>
          )}
        </div>
      ) : (
        <div className="navbar__item">
          <div className="button-group">
            <a href="/auth" className="button button--secondary button--sm" style={{ marginRight: '0.5rem' }}>
              Login
            </a>
            <a href="/auth" onClick={() => setIsLogin(false)} className="button button--primary button--sm">
              Signup
            </a>
          </div>
        </div>
      )}

      {/* Auth Modal */}
      {showAuthModal && (
        <div className="modal-backdrop" style={{
          position: 'fixed',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          backgroundColor: 'rgba(0,0,0,0.5)',
          zIndex: 1000,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center'
        }} onClick={() => setShowAuthModal(false)}>
          <div style={{
            backgroundColor: 'white',
            padding: '2rem',
            borderRadius: '8px',
            maxWidth: '400px',
            width: '90%',
            position: 'relative'
          }} onClick={(e) => e.stopPropagation()}>
            <h3>{isLogin ? 'Login' : 'Register'}</h3>

            <form onSubmit={isLogin ? handleLogin : handleRegister}>
              <div style={{ marginBottom: '1rem' }}>
                <input
                  type="email"
                  placeholder="Email"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                  style={{
                    width: '100%',
                    padding: '0.5rem',
                    border: '1px solid #ccc',
                    borderRadius: '4px'
                  }}
                />
              </div>

              <div style={{ marginBottom: '1rem' }}>
                <input
                  type="password"
                  placeholder="Password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                  style={{
                    width: '100%',
                    padding: '0.5rem',
                    border: '1px solid #ccc',
                    borderRadius: '4px'
                  }}
                />
              </div>

              {!isLogin && (
                <>
                  <div style={{ marginBottom: '1rem' }}>
                    <select
                      value={softwareBackground}
                      onChange={(e) => setSoftwareBackground(e.target.value)}
                      style={{
                        width: '100%',
                        padding: '0.5rem',
                        border: '1px solid #ccc',
                        borderRadius: '4px'
                      }}
                      required
                    >
                      <option value="">Select software background</option>
                      <option value="web_dev">Web Development</option>
                      <option value="backend">Backend Development</option>
                      <option value="ai_ml">AI/ML</option>
                      <option value="beginner">Beginner</option>
                      <option value="intermediate">Intermediate</option>
                      <option value="advanced">Advanced</option>
                    </select>
                  </div>

                  <div style={{ marginBottom: '1rem' }}>
                    <input
                      type="text"
                      placeholder="Hardware background (e.g., Arduino, ROS)"
                      value={hardwareBackground}
                      onChange={(e) => setHardwareBackground(e.target.value)}
                      style={{
                        width: '100%',
                        padding: '0.5rem',
                        border: '1px solid #ccc',
                        borderRadius: '4px'
                      }}
                    />
                  </div>

                  <div style={{ marginBottom: '1rem' }}>
                    <input
                      type="text"
                      placeholder="Learning goal"
                      value={learningGoal}
                      onChange={(e) => setLearningGoal(e.target.value)}
                      style={{
                        width: '100%',
                        padding: '0.5rem',
                        border: '1px solid #ccc',
                        borderRadius: '4px'
                      }}
                    />
                  </div>
                </>
              )}

              <div style={{ display: 'flex', gap: '0.5rem' }}>
                <button
                  type="submit"
                  className="button button--primary"
                  style={{
                    flex: 1,
                    padding: '0.5rem'
                  }}
                >
                  {isLogin ? 'Login' : 'Register'}
                </button>

                <button
                  type="button"
                  className="button button--secondary"
                  onClick={() => setIsLogin(!isLogin)}
                  style={{
                    flex: 1,
                    padding: '0.5rem'
                  }}
                >
                  {isLogin ? 'Need an account?' : 'Already have an account?'}
                </button>
              </div>
            </form>

            <button
              onClick={() => setShowAuthModal(false)}
              style={{
                position: 'absolute',
                top: '1rem',
                right: '1rem',
                background: 'none',
                border: 'none',
                fontSize: '1.2rem',
                cursor: 'pointer'
              }}
            >
              ×
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default NavbarAuth;