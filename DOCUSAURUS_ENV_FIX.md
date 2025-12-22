# Docusaurus Environment Variable Fix

## Issues Fixed

### 1. "process is not defined" Error
**Problem:** Docusaurus client-side code cannot access `process.env` like Next.js can.

**Solution:** Created a centralized environment configuration system:
1. Created `src/config/env.config.js` - Central config module
2. Created `static/js/env-loader.js` - Loads env vars into window.env
3. Updated `docusaurus.config.js` - Added customFields and env-loader script
4. Replaced all `process.env.NEXT_PUBLIC_*` usage with `ENV_CONFIG` imports

### 2. Chatbot Widget Lazy Loading
**Problem:** Widget loads before React is ready, causing "Cannot read properties of undefined (reading 'lazy')" error.

**Solution:**
- Added `env-loader.js` to load BEFORE chatbot widget
- Set `async: false, defer: false` for env-loader to ensure synchronous loading
- Widget now has access to environment variables via `window.env`

## Files Modified

### New Files Created
1. **src/config/env.config.js** - Centralized environment configuration
   - Reads from window.env (set by env-loader.js)
   - Reads from Docusaurus customFields
   - Provides fallback defaults

2. **static/js/env-loader.js** - Client-side environment variable loader
   - Loads FIRST before any other scripts
   - Creates window.env object
   - Sets defaults for local development

### Files Updated
1. **docusaurus.config.js**
   - Added `customFields` with authBackendUrl, ragBackendUrl, frontendUrl
   - Added env-loader.js script (loads first, synchronously)
   - Script order: env-loader → React → ReactDOM → chatbot-widget

2. **src/auth/context/AuthContext.js**
   - Removed `process.env.NEXT_PUBLIC_AUTH_BACKEND_URL`
   - Added `import ENV_CONFIG from '../../config/env.config'`
   - Uses `ENV_CONFIG.AUTH_BACKEND_URL`

3. **src/auth/client.ts**
   - Removed `process.env.NEXT_PUBLIC_AUTH_BACKEND_URL`
   - Added `import ENV_CONFIG from '../config/env.config'`
   - Uses `ENV_CONFIG.AUTH_BACKEND_URL`

4. **src/auth/api/api-client.ts**
   - Removed `process.env.NEXT_PUBLIC_AUTH_BACKEND_URL`
   - Added `import ENV_CONFIG from '../../config/env.config'`
   - Uses `ENV_CONFIG.AUTH_BACKEND_URL`

5. **src/theme/Root.tsx**
   - Removed `process.env.NEXT_PUBLIC_RAG_BACKEND_URL`
   - Added `import ENV_CONFIG from '../config/env.config'`
   - Uses `ENV_CONFIG.RAG_BACKEND_URL`

6. **src/personalization/services/personalization.service.ts**
   - Removed `process.env.NEXT_PUBLIC_RAG_BACKEND_URL`
   - Uses dynamic `import('../../config/env.config')`
   - Uses `ENV_CONFIG.default.RAG_BACKEND_URL`

7. **src/personalization/components/PersonalizeChapterButton.js**
   - Removed `process.env.NEXT_PUBLIC_RAG_BACKEND_URL`
   - Added `import ENV_CONFIG from '../../config/env.config'`
   - Uses `ENV_CONFIG.RAG_BACKEND_URL`

## How It Works

### Script Loading Order
```
1. env-loader.js (sync)      ← Loads first, creates window.env
2. React (async)              ← React library
3. ReactDOM (async)           ← ReactDOM library
4. chatbot-widget.js (async)  ← Widget can now access window.env
```

### Configuration Resolution Priority
```
1. window.env.NEXT_PUBLIC_* (set by env-loader.js from meta tags)
2. window.docusaurus.customFields.* (from docusaurus.config.js)
3. Hardcoded defaults (http://localhost:8000, etc.)
```

## Environment Variables

### Development (.env)
```bash
NEXT_PUBLIC_AUTH_BACKEND_URL=http://localhost:8000
NEXT_PUBLIC_RAG_BACKEND_URL=http://localhost:8000
NEXT_PUBLIC_FRONTEND_URL=http://localhost:3002
```

### Production
```bash
NEXT_PUBLIC_AUTH_BACKEND_URL=https://your-auth-backend.vercel.app
NEXT_PUBLIC_RAG_BACKEND_URL=https://your-rag-backend.vercel.app
NEXT_PUBLIC_FRONTEND_URL=https://your-frontend.vercel.app
```

## Testing

1. **Clear browser cache** and reload
2. **Open console** - should see `[ENV] Environment variables loaded:`
3. **Test authentication** - login/signup should work
4. **Test chatbot** - should load without errors
5. **Test personalization** - should connect to RAG backend

## Notes

- Docusaurus is NOT Next.js - it requires a different approach for client-side env vars
- `process.env` only works in Node.js server-side code
- Browser code must use `window.env` or imported ENV_CONFIG
- The env-loader.js script bridges the gap between build-time and runtime configuration

## Troubleshooting

If you see "process is not defined":
- Check that env-loader.js is loading first in docusaurus.config.js
- Check browser console for ENV loading logs
- Verify ENV_CONFIG import in the file causing the error

If chatbot doesn't load:
- Check React/ReactDOM are loading correctly
- Check browser console for "lazy" errors
- Verify chatbot-widget.js is last in script order
