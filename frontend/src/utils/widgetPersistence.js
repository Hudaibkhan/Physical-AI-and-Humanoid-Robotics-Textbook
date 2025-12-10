/**
 * Helper functions for managing widget persistence across page navigation
 */

// Store widget state in sessionStorage to persist across page refreshes/navigation
const WIDGET_STATE_KEY = 'chatbot_widget_state';

/**
 * Save widget state to sessionStorage
 * @param {Object} state - The current state of the widget
 */
export const saveWidgetState = (state) => {
  try {
    const widgetState = {
      ...state,
      timestamp: Date.now() // Add timestamp for potential expiration
    };
    sessionStorage.setItem(WIDGET_STATE_KEY, JSON.stringify(widgetState));
  } catch (error) {
    console.warn('Could not save widget state to sessionStorage:', error);
  }
};

/**
 * Load widget state from sessionStorage
 * @returns {Object|null} The saved widget state or null if not found/expired
 */
export const loadWidgetState = () => {
  try {
    const savedState = sessionStorage.getItem(WIDGET_STATE_KEY);
    if (savedState) {
      const parsedState = JSON.parse(savedState);

      // Check if state is expired (older than 1 hour)
      const now = Date.now();
      const oneHour = 60 * 60 * 1000;

      if (now - parsedState.timestamp < oneHour) {
        return parsedState;
      } else {
        // State is expired, remove it
        sessionStorage.removeItem(WIDGET_STATE_KEY);
        return null;
      }
    }
    return null;
  } catch (error) {
    console.warn('Could not load widget state from sessionStorage:', error);
    return null;
  }
};

/**
 * Clear saved widget state
 */
export const clearWidgetState = () => {
  try {
    sessionStorage.removeItem(WIDGET_STATE_KEY);
  } catch (error) {
    console.warn('Could not clear widget state from sessionStorage:', error);
  }
};

/**
 * Listen for page navigation events and handle widget persistence
 * @param {Function} onNavigate - Callback function to handle navigation
 * @returns {Function} Cleanup function to remove event listeners
 */
export const setupNavigationHandler = (onNavigate) => {
  // Handle browser back/forward buttons
  const handlePopState = () => {
    onNavigate();
  };

  // Handle hash changes (common in SPAs)
  const handleHashChange = () => {
    onNavigate();
  };

  // Handle custom route changes (for React/Vue apps)
  const handleCustomRouteChange = () => {
    onNavigate();
  };

  // Handle page unload to save state
  const handleBeforeUnload = () => {
    // State is automatically saved in component state updates
  };

  // Add event listeners
  window.addEventListener('popstate', handlePopState);
  window.addEventListener('hashchange', handleHashChange);
  window.addEventListener('routeChange', handleCustomRouteChange); // Custom event
  window.addEventListener('beforeunload', handleBeforeUnload);

  // Also handle Turbolinks (Rails) and similar SPA navigation patterns
  window.addEventListener('turbolinks:before-visit', handleBeforeUnload);
  window.addEventListener('turbolinks:load', handlePopState);

  // For React Router v5+
  window.addEventListener('historyPush', handlePopState);
  window.addEventListener('historyReplace', handlePopState);

  // Return cleanup function
  return () => {
    window.removeEventListener('popstate', handlePopState);
    window.removeEventListener('hashchange', handleHashChange);
    window.removeEventListener('routeChange', handleCustomRouteChange);
    window.removeEventListener('beforeunload', handleBeforeUnload);
    window.removeEventListener('turbolinks:before-visit', handleBeforeUnload);
    window.removeEventListener('turbolinks:load', handlePopState);
    window.removeEventListener('historyPush', handlePopState);
    window.removeEventListener('historyReplace', handlePopState);
  };
};

/**
 * Check if the widget should remain open after navigation
 * @param {Object} currentState - Current widget state
 * @param {Object} savedState - Previously saved state
 * @returns {boolean} Whether the widget should remain open
 */
export const shouldKeepWidgetOpen = (currentState, savedState) => {
  if (!savedState) return false;

  // If the widget was open before navigation, keep it open
  if (savedState.isOpen) {
    // Check if it's been less than 10 minutes since last activity
    const tenMinutes = 10 * 60 * 1000;
    return (Date.now() - savedState.timestamp) < tenMinutes;
  }

  return false;
};

/**
 * Initialize widget with persisted state
 * @returns {Object} Initial state for the widget
 */
export const initializeWidgetWithPersistedState = () => {
  const savedState = loadWidgetState();

  if (savedState) {
    return {
      isOpen: shouldKeepWidgetOpen({ isOpen: false }, savedState),
      isWidgetVisible: true,
      messages: savedState.messages || [],
      selectedText: savedState.selectedText || '',
      sessionId: savedState.sessionId || null
    };
  }

  // Default initial state
  return {
    isOpen: false,
    isWidgetVisible: true,
    messages: [],
    selectedText: '',
    sessionId: null
  };
};