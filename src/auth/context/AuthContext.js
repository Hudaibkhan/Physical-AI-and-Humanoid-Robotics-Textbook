import React, { createContext, useContext, useReducer, useEffect } from 'react';
import ENV_CONFIG from '../../config/env.config';

// Get backend URL from environment configuration
const AUTH_BACKEND_URL = ENV_CONFIG.AUTH_BACKEND_URL;

// Initial state
const initialState = {
  user: null,
  token: null,
  isAuthenticated: false,
  loading: true,
};

// Auth context
const AuthContext = createContext();

// Reducer for auth state
const authReducer = (state, action) => {
  switch (action.type) {
    case 'LOGIN_START':
      return { ...state, loading: true };
    case 'LOGIN_SUCCESS':
      return {
        ...state,
        user: action.payload.user,
        token: action.payload.token,
        isAuthenticated: true,
        loading: false,
      };
    case 'LOGIN_FAILURE':
      return { ...state, loading: false };
    case 'REGISTER_SUCCESS':
      return {
        ...state,
        user: action.payload.user,
        token: action.payload.token,
        isAuthenticated: true,
        loading: false,
      };
    case 'LOGOUT':
      return {
        ...state,
        user: null,
        token: null,
        isAuthenticated: false,
        loading: false,
      };
    case 'FETCH_USER_SUCCESS':
      return {
        ...state,
        user: action.payload.user,
        isAuthenticated: true,
        loading: false,
      };
    case 'SET_LOADING':
      return { ...state, loading: action.payload };
    default:
      return state;
  }
};

// Auth provider component
export const AuthProvider = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, initialState);

  // Check for existing session on app load
  // ALWAYS call fetchUser() - session is in httpOnly cookie, not localStorage
  useEffect(() => {
    fetchUser();
  }, []);

  // Login function
  const login = async (email, password) => {
    dispatch({ type: 'LOGIN_START' });

    try {
      // Call Better Auth sign-in endpoint
      const response = await fetch(`${AUTH_BACKEND_URL}/api/auth/sign-in/email`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',  // Important: receive and send cookies
        body: JSON.stringify({ email, password })
      });

      // Check if response has content before parsing JSON
      const contentType = response.headers.get('content-type');
      let data = null;

      if (contentType && contentType.includes('application/json')) {
        const text = await response.text();
        if (text && text.trim().length > 0) {
          try {
            data = JSON.parse(text);
          } catch (parseError) {
            console.error('Failed to parse JSON response:', text);
            throw new Error('Invalid server response format');
          }
        }
      }

      if (response.ok && data?.user) {
        const { user, session } = data;
        // Session stored in httpOnly cookie automatically
        dispatch({ type: 'LOGIN_SUCCESS', payload: { token: session?.token, user } });
        return;
      } else {
        const errorMessage = data?.error || data?.message || `Login failed (${response.status})`;
        throw new Error(errorMessage);
      }
    } catch (error) {
      dispatch({ type: 'LOGIN_FAILURE' });
      console.error('Login error:', error);
      throw error;
    }
  };

  // Register function
  const register = async (
    name,
    email,
    password,
    skill_level,
    software_background,
    hardware_background,
    learning_goal
  ) => {
    dispatch({ type: 'LOGIN_START' });

    try {
      // Call Better Auth sign-up endpoint
      // IMPORTANT: Only send Better Auth fields (name, email, password)
      // Profile fields are saved separately via /api/user/profile/create
      const response = await fetch(`${AUTH_BACKEND_URL}/api/auth/sign-up/email`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',  // Important: send cookies
        body: JSON.stringify({
          name,
          email,
          password
          // Do NOT send profile fields to Better Auth - it ignores them
        })
      });

      // Check if response has content before parsing JSON
      const contentType = response.headers.get('content-type');
      let data = null;

      if (contentType && contentType.includes('application/json')) {
        const text = await response.text();
        if (text && text.trim().length > 0) {
          try {
            data = JSON.parse(text);
          } catch (parseError) {
            console.error('Failed to parse JSON response:', text);
            throw new Error('Invalid server response format');
          }
        }
      }

      if (response.ok && data?.user) {
        // Better Auth returns user and session in the response
        const { user, session } = data;

        // CRITICAL: Create user profile in separate table
        // Must call /create endpoint (not PUT /profile)
        if (skill_level || software_background || hardware_background || learning_goal) {
          try {
            console.log('[AUTH] Creating user profile...', {
              skillLevel: skill_level,
              softwareBackground: software_background,
              hardwareBackground: hardware_background,
              learningGoal: learning_goal
            });

            const profileResponse = await fetch(`${AUTH_BACKEND_URL}/api/user/profile/create`, {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              credentials: 'include',  // Session cookie from signup
              body: JSON.stringify({
                skillLevel: skill_level,              // Backend expects camelCase
                softwareBackground: software_background,
                hardwareBackground: hardware_background,
                learningGoal: learning_goal
              })
            });

            console.log('[AUTH] Profile creation response status:', profileResponse.status);

            if (!profileResponse.ok) {
              const errorText = await profileResponse.text();
              console.error('[AUTH] Profile creation failed:', errorText);
              // Don't fail signup - profile can be created later
            } else {
              const profileData = await profileResponse.json();
              console.log('[AUTH] ✅ Profile created successfully:', profileData);
            }
          } catch (profileError) {
            console.error('[AUTH] Failed to save profile data:', profileError);
            // Don't fail signup if profile save fails
          }
        } else {
          console.warn('[AUTH] No profile data provided during signup');
        }

        // Merge profile data with user for immediate display
        const userWithProfile = {
          ...user,
          skill_level,
          software_background,
          hardware_background,
          learning_goal
        };

        dispatch({ type: 'REGISTER_SUCCESS', payload: { token: session?.token, user: userWithProfile } });
        return;
      } else {
        const errorMessage = data?.error || data?.message || `Registration failed (${response.status})`;
        throw new Error(errorMessage);
      }
    } catch (error) {
      dispatch({ type: 'LOGIN_FAILURE' });
      console.error('Registration error:', error);
      throw error;
    }
  };

  // Logout function
  const logout = async () => {
    try {
      // Call Better Auth sign-out endpoint
      await fetch(`${AUTH_BACKEND_URL}/api/auth/sign-out`, {
        method: 'POST',
        credentials: 'include'  // Important: send cookies
      });

      // Clear state - cookie is cleared by server
      dispatch({ type: 'LOGOUT' });
    } catch (error) {
      console.error('Logout error:', error);
      // Still clear local state even if API call fails
      dispatch({ type: 'LOGOUT' });
    }
  };

  // Fetch user function - gets session and profile data
  const fetchUser = async () => {
    try {
      console.log('[AUTH] Checking session...');

      // First, check for active session
      const sessionResponse = await fetch(`${AUTH_BACKEND_URL}/api/auth/get-session`, {
        method: 'GET',
        credentials: 'include',  // CRITICAL: Must send cookies
        headers: {
          'Content-Type': 'application/json'
        }
      });

      console.log('[AUTH] Session response status:', sessionResponse.status);

      if (!sessionResponse.ok) {
        console.log('[AUTH] No active session found');
        dispatch({ type: 'SET_LOADING', payload: false });
        return;
      }

      const sessionData = await sessionResponse.json();
      console.log('[AUTH] Session data:', sessionData);

      if (!sessionData || !sessionData.user) {
        console.log('[AUTH] No user in session');
        dispatch({ type: 'SET_LOADING', payload: false });
        return;
      }

      console.log('[AUTH] Session restored for user:', sessionData.user.email);

      // Fetch profile data to merge with user
      try {
        const profileResponse = await fetch(`${AUTH_BACKEND_URL}/api/user/profile`, {
          credentials: 'include'
        });

        if (profileResponse.ok) {
          const profileData = await profileResponse.json();
          // Merge user with profile data
          const mergedUser = {
            ...sessionData.user,
            ...profileData.profile
          };
          dispatch({ type: 'FETCH_USER_SUCCESS', payload: { user: mergedUser } });
        } else {
          // Profile fetch failed but user exists - use session data only
          dispatch({ type: 'FETCH_USER_SUCCESS', payload: { user: sessionData.user } });
        }
      } catch (profileError) {
        console.error('Profile fetch error:', profileError);
        // Use session data without profile
        dispatch({ type: 'FETCH_USER_SUCCESS', payload: { user: sessionData.user } });
      }
    } catch (error) {
      console.error('Fetch user error:', error);
      dispatch({ type: 'SET_LOADING', payload: false });
    }
  };

  const value = {
    state,
    login,
    register,
    logout,
    fetchUser,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

// Custom hook to use auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export default AuthContext;