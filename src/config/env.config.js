/**
 * Environment Configuration for Docusaurus
 *
 * Since Docusaurus doesn't support process.env in browser code,
 * we use a combination of:
 * 1. Custom fields from docusaurus.config.js
 * 2. Window global variables
 * 3. Fallback defaults
 */

// Try to get config from Docusaurus custom fields
const getDocusaurusConfig = () => {
  if (typeof window !== 'undefined') {
    try {
      // Docusaurus exposes custom fields via window.docusaurus
      return window.docusaurus?.customFields || {};
    } catch (e) {
      return {};
    }
  }
  return {};
};

// Try to get config from window.env (set by script tag)
const getWindowEnv = () => {
  if (typeof window !== 'undefined' && window.env) {
    return window.env;
  }
  return {};
};

const docusaurusConfig = getDocusaurusConfig();
const windowEnv = getWindowEnv();

// Export configuration with production-ready fallbacks (NO LOCALHOST)
export const ENV_CONFIG = {
  // Auth Backend URL (Better Auth - Node.js)
  AUTH_BACKEND_URL:
    windowEnv.NEXT_PUBLIC_AUTH_BACKEND_URL ||
    docusaurusConfig.authBackendUrl ||
    'https://physical-ai-book-database.vercel.app',

  // RAG Backend URL (FastAPI - Python)
  RAG_BACKEND_URL:
    windowEnv.NEXT_PUBLIC_RAG_BACKEND_URL ||
    docusaurusConfig.ragBackendUrl ||
    'https://fastapi-backend-for-book.vercel.app',

  // Frontend URL
  FRONTEND_URL:
    windowEnv.NEXT_PUBLIC_FRONTEND_URL ||
    docusaurusConfig.frontendUrl ||
    'https://physical-ai-and-humanoid-robotics-t-lake.vercel.app',

  // Database URL (server-side only, but keeping for completeness)
  DATABASE_URL:
    docusaurusConfig.databaseUrl ||
    '',

  // Better Auth Secret (server-side only)
  BETTER_AUTH_SECRET:
    docusaurusConfig.betterAuthSecret ||
    '',
};

// Log configuration in development
if (typeof window !== 'undefined' && window.location.hostname === 'localhost') {
  console.log('[ENV_CONFIG] Loaded configuration:', {
    AUTH_BACKEND_URL: ENV_CONFIG.AUTH_BACKEND_URL,
    RAG_BACKEND_URL: ENV_CONFIG.RAG_BACKEND_URL,
    FRONTEND_URL: ENV_CONFIG.FRONTEND_URL,
  });
}

export default ENV_CONFIG;
