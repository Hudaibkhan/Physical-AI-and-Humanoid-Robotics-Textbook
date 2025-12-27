import React, { createContext, useContext, useReducer, useEffect } from 'react';
import ENV_CONFIG from '../../config/env.config';

// CRITICAL FIX: Use relative path for auth API to leverage Vercel rewrites
// This makes cookies first-party (same domain) instead of third-party (cross-domain)
// Vercel rewrites /auth-api/* to the backend, but cookies are set on frontend domain
const getAuthApiUrl = () => {
  if (typeof window !== 'undefined') {
    // In production (Vercel), use relative path for proxy
    // This avoids cross-domain cookie issues since vercel.app is on the Public Suffix List
    if (window.location.hostname.includes('vercel.app')) {
      return ''; // Use relative URLs like /auth-api/auth/sign-in
    }
  }
  // In development or custom domains, use the direct backend URL
  return ENV_CONFIG.AUTH_BACKEND_URL;
};

const AUTH_BACKEND_URL = getAuthApiUrl();
const AUTH_API_PREFIX = AUTH_BACKEND_URL ? `${AUTH_BACKEND_URL}/api` : '/auth-api';

// Debug log to help troubleshoot auth issues
if (typeof window !== 'undefined') {
  console.log('[AUTH] API configuration:', {
    hostname: window.location.hostname,
    isVercel: window.location.hostname.includes('vercel.app'),
    AUTH_API_PREFIX,
    usingProxy: AUTH_API_PREFIX === '/auth-api'
  });
}

/* =======================
   Initial State
======================= */
const initialState = {
  user: null,
  token: null,
  isAuthenticated: false,
  loading: true, // VERY IMPORTANT
};

/* =======================
   Context
======================= */
const AuthContext = createContext();

/* =======================
   Reducer
======================= */
const authReducer = (state, action) => {
  switch (action.type) {
    case 'LOGIN_START':
      return { ...state, loading: true };

    case 'LOGIN_SUCCESS':
    case 'REGISTER_SUCCESS':
      return {
        ...state,
        user: action.payload.user,
        token: action.payload.token || null,
        isAuthenticated: true,
        loading: false,
      };

    case 'FETCH_USER_SUCCESS':
      return {
        ...state,
        user: action.payload.user,
        isAuthenticated: true,
        loading: false,
      };

    case 'NO_SESSION':
      return {
        ...state,
        user: null,
        token: null,
        isAuthenticated: false,
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

    case 'SET_LOADING':
      return { ...state, loading: action.payload };

    default:
      return state;
  }
};

/* =======================
   Provider
======================= */
export const AuthProvider = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, initialState);

  /* =======================
     Restore session on load
  ======================= */
  useEffect(() => {
    fetchUser();
  }, []);

  /* =======================
     Login
  ======================= */
  const login = async (email, password) => {
    console.log('[AUTH] Login attempt for:', email);
    dispatch({ type: 'LOGIN_START' });

    try {
      const res = await fetch(
        `${AUTH_API_PREFIX}/auth/sign-in/email`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          credentials: 'include',
          body: JSON.stringify({ email, password }),
        }
      );

      const data = await res.json();

      if (!res.ok || !data?.user) {
        console.error('[AUTH] Login failed:', data?.message || 'Unknown error');
        throw new Error(data?.message || 'Login failed');
      }

      console.log('[AUTH] Login successful for:', data.user.email);
      dispatch({
        type: 'LOGIN_SUCCESS',
        payload: {
          user: data.user,
          token: data.session?.token,
        },
      });
    } catch (err) {
      dispatch({ type: 'NO_SESSION' });
      throw err;
    }
  };

  /* =======================
     Register + Profile Save
  ======================= */
  const register = async (
    name,
    email,
    password,
    skill_level,
    software_background,
    hardware_background,
    learning_goal
  ) => {
    console.log('[AUTH] Signup attempt for:', email);
    dispatch({ type: 'LOGIN_START' });

    try {
      /* ---- Step 1: Signup ---- */
      console.log('[AUTH] Step 1: Creating user account...');
      const res = await fetch(
        `${AUTH_API_PREFIX}/auth/sign-up/email`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          credentials: 'include',
          body: JSON.stringify({ name, email, password }),
        }
      );

      const data = await res.json();

      if (!res.ok || !data?.user) {
        console.error('[AUTH] Signup failed:', data?.message || 'Unknown error');
        throw new Error(data?.message || 'Signup failed');
      }

      console.log('[AUTH] User account created:', data.user.email);

      /* ---- Step 2: Wait for cookie to be ready ---- */
      console.log('[AUTH] Step 2: Waiting for session cookie...');
      await new Promise((r) => setTimeout(r, 200));

      /* ---- Step 3: Confirm session exists ---- */
      console.log('[AUTH] Step 3: Verifying session...');
      const sessionCheck = await fetch(
        `${AUTH_API_PREFIX}/auth/get-session`,
        { credentials: 'include' }
      );

      // CRITICAL FIX: Parse response body to verify session actually exists
      // Better Auth returns HTTP 200 with null body when no session exists
      const sessionData = await sessionCheck.json().catch(() => null);

      if (!sessionCheck.ok || !sessionData?.user) {
        console.error('[AUTH] Session not ready:', {
          httpStatus: sessionCheck.status,
          hasUser: !!sessionData?.user,
          sessionData: sessionData
        });
        console.error('[AUTH] Skipping profile creation - no valid session');
      } else {
        console.log('[AUTH] Session verified successfully for user:', sessionData.user.email);
        /* ---- Step 4: Create profile ---- */
        try {
          console.log('[AUTH] Creating user profile...');
          const profileRes = await fetch(
            `${AUTH_API_PREFIX}/user/profile/create`,
            {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              credentials: 'include',
              body: JSON.stringify({
                skillLevel: skill_level,
                softwareBackground: software_background,
                hardwareBackground: hardware_background,
                learningGoal: learning_goal,
              }),
            }
          );

          if (!profileRes.ok) {
            const profileError = await profileRes.json().catch(() => ({}));
            console.error('[AUTH] Profile creation failed:', {
              status: profileRes.status,
              statusText: profileRes.statusText,
              error: profileError
            });
          } else {
            console.log('[AUTH] Profile created successfully');
          }
        } catch (profileError) {
          console.error('[AUTH] Profile creation error:', profileError);
          // Don't throw - signup succeeded, profile creation is non-critical
        }
      }

      dispatch({
        type: 'REGISTER_SUCCESS',
        payload: {
          user: {
            ...data.user,
            skill_level,
            software_background,
            hardware_background,
            learning_goal,
          },
          token: data.session?.token,
        },
      });
    } catch (err) {
      dispatch({ type: 'NO_SESSION' });
      throw err;
    }
  };

  /* =======================
     Logout
  ======================= */
  const logout = async () => {
    try {
      await fetch(
        `${AUTH_API_PREFIX}/auth/sign-out`,
        {
          method: 'POST',
          credentials: 'include',
        }
      );
    } catch (_) {}

    dispatch({ type: 'LOGOUT' });
  };

  /* =======================
     Fetch User (Session Restore)
  ======================= */
  const fetchUser = async () => {
    try {
      console.log('[AUTH] Checking for existing session...');

      // Add timeout to prevent infinite loading
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 10000); // 10 second timeout

      const sessionRes = await fetch(
        `${AUTH_API_PREFIX}/auth/get-session`,
        {
          credentials: 'include',
          signal: controller.signal
        }
      );

      clearTimeout(timeoutId);

      if (!sessionRes.ok) {
        console.log('[AUTH] No session found (HTTP', sessionRes.status, ')');
        dispatch({ type: 'NO_SESSION' });
        return;
      }

      const sessionData = await sessionRes.json();

      if (!sessionData?.user) {
        console.log('[AUTH] Session response missing user data');
        dispatch({ type: 'NO_SESSION' });
        return;
      }

      console.log('[AUTH] Session restored for user:', sessionData.user.email);

      /* ---- Fetch profile ---- */
      let profile = {};
      try {
        const profileRes = await fetch(
          `${AUTH_API_PREFIX}/user/profile`,
          { credentials: 'include' }
        );

        if (profileRes.ok) {
          const profileData = await profileRes.json();
          profile = profileData.profile || {};
          console.log('[AUTH] Profile loaded successfully');
        } else {
          console.log('[AUTH] No profile found (will be created on next update)');
        }
      } catch (profileError) {
        console.error('[AUTH] Profile fetch error:', profileError);
      }

      dispatch({
        type: 'FETCH_USER_SUCCESS',
        payload: {
          user: {
            ...sessionData.user,
            skill_level: profile.skill_level,
            software_background: profile.software_background,
            hardware_background: profile.hardware_background,
            learning_goal: profile.learning_goal,
          },
        },
      });
    } catch (err) {
      if (err.name === 'AbortError') {
        console.error('[AUTH] Session check timed out - backend may be down');
      } else {
        console.error('[AUTH] Session restore failed:', err);
      }
      dispatch({ type: 'NO_SESSION' });
    }
  };

  return (
    <AuthContext.Provider
      value={{
        state,
        login,
        register,
        logout,
        fetchUser,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};

/* =======================
   Hook
======================= */
export const useAuth = () => {
  const ctx = useContext(AuthContext);
  if (!ctx) throw new Error('useAuth must be used inside AuthProvider');
  return ctx;
};

export default AuthContext;
