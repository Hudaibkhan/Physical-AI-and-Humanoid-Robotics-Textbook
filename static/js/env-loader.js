/**
 * Environment Variable Loader for Docusaurus
 *
 * This script loads environment variables and makes them available client-side.
 * It should be loaded BEFORE any other scripts that need env variables.
 */

(function() {
  // Create window.env object if it doesn't exist
  if (typeof window !== 'undefined') {
    window.env = window.env || {};

    // Set environment variables from meta tags (injected during build)
    const authBackendMeta = document.querySelector('meta[name="auth-backend-url"]');
    const ragBackendMeta = document.querySelector('meta[name="rag-backend-url"]');
    const frontendUrlMeta = document.querySelector('meta[name="frontend-url"]');

    if (authBackendMeta) {
      window.env.NEXT_PUBLIC_AUTH_BACKEND_URL = authBackendMeta.content;
    }

    if (ragBackendMeta) {
      window.env.NEXT_PUBLIC_RAG_BACKEND_URL = ragBackendMeta.content;
    }

    if (frontendUrlMeta) {
      window.env.NEXT_PUBLIC_FRONTEND_URL = frontendUrlMeta.content;
    }

    // Fallback defaults for local development
    window.env.NEXT_PUBLIC_AUTH_BACKEND_URL = window.env.NEXT_PUBLIC_AUTH_BACKEND_URL || 'http://localhost:8000';
    window.env.NEXT_PUBLIC_RAG_BACKEND_URL = window.env.NEXT_PUBLIC_RAG_BACKEND_URL || 'http://localhost:8000';
    window.env.NEXT_PUBLIC_FRONTEND_URL = window.env.NEXT_PUBLIC_FRONTEND_URL || 'http://localhost:3002';

    console.log('[ENV] Environment variables loaded:', {
      AUTH_BACKEND_URL: window.env.NEXT_PUBLIC_AUTH_BACKEND_URL,
      RAG_BACKEND_URL: window.env.NEXT_PUBLIC_RAG_BACKEND_URL,
      FRONTEND_URL: window.env.NEXT_PUBLIC_FRONTEND_URL,
    });
  }
})();
