import React, { useState, useEffect, lazy, Suspense } from 'react';
import { addTextSelectionListenerWithVisualFeedback } from '../utils/textSelection';
import {
  initializeWidgetWithPersistedState,
  saveWidgetState,
  setupNavigationHandler
} from '../utils/widgetPersistence';
import '../styles/chatbot.css';

// Lazy load the ChatModal component to reduce initial bundle size
const LazyChatModal = lazy(() => import('./ChatModal'));

const ChatWidget = ({ backendUrl }) => {
  // Initialize with persisted state
  const initialState = initializeWidgetWithPersistedState();
  const [isOpen, setIsOpen] = useState(initialState.isOpen);
  const [isWidgetVisible, setIsWidgetVisible] = useState(initialState.isWidgetVisible);
  const [isInitialized, setIsInitialized] = useState(false);

  // Check if we're on a book page and initialize
  useEffect(() => {
    // Initialize the chat widget
    const init = () => {
      // Check if widget should be visible on this page
      // For now, we'll show it on all pages
      setIsWidgetVisible(true);
    };

    init();

    // Add text selection listener with visual feedback
    const removeTextSelectionListener = addTextSelectionListenerWithVisualFeedback((selection) => {
      // This callback can be used to handle text selection if needed
      // For now, we're just using the visual feedback and custom event
    });

    // Handle page navigation to maintain widget state
    const handleLocationChange = () => {
      // Preserve widget visibility across navigation
      setIsWidgetVisible(true);

      // If modal is open, keep it open across navigation
      // (though this may need to be handled differently depending on the use case)
    };

    // Listen for browser navigation events
    window.addEventListener('popstate', handleLocationChange);
    window.addEventListener('hashchange', handleLocationChange);

    // Also listen for custom navigation events (common in SPAs)
    window.addEventListener('routeChange', handleLocationChange);

    return () => {
      removeTextSelectionListener();
      window.removeEventListener('popstate', handleLocationChange);
      window.removeEventListener('hashchange', handleLocationChange);
      window.removeEventListener('routeChange', handleLocationChange);
    };
  }, []);

  // Handle widget state persistence across page navigation
  useEffect(() => {
    // Set up navigation handlers for persistence
    const cleanupNavigation = setupNavigationHandler(() => {
      // On navigation, save the current state
      saveWidgetState({
        isOpen,
        isWidgetVisible,
        timestamp: Date.now()
      });
    });

    // Save state when component updates
    saveWidgetState({
      isOpen,
      isWidgetVisible,
      timestamp: Date.now()
    });

    return () => {
      // Clean up navigation handlers
      cleanupNavigation();

      // Save final state before unmounting
      saveWidgetState({
        isOpen,
        isWidgetVisible,
        timestamp: Date.now()
      });
    };
  }, [isOpen, isWidgetVisible]);

  // Load ChatModal only when needed
  useEffect(() => {
    if (isOpen && !isInitialized) {
      setIsInitialized(true);
    }
  }, [isOpen, isInitialized]);

  if (!isWidgetVisible) {
    return null;
  }

  return (
    <div className="chatbot-container" role="region" aria-label="Chatbot widget">
      {isOpen && (
        <Suspense fallback={
          <div className="chatbot-modal-loading" role="status" aria-live="polite">
            Loading chat...
          </div>
        }>
          <LazyChatModal backendUrl={backendUrl} onClose={() => setIsOpen(false)} />
        </Suspense>
      )}
      <button
        className="chatbot-float-button"
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? "Close chatbot" : "Open chatbot"}
        aria-expanded={isOpen}
        aria-controls="chatbot-modal"
      >
        <div className="chatbot-icon" aria-hidden="true">💬</div>
      </button>
    </div>
  );
};

// Initialize function for use in HTML pages
ChatWidget.init = (config = {}) => {
  const defaultConfig = {
    backendUrl: 'https://fastapi-backend-for-book.vercel.app',
    theme: 'auto' // 'light', 'dark', or 'auto'
  };

  const finalConfig = { ...defaultConfig, ...config };

  // Create a container for the chat widget
  const container = document.createElement('div');
  container.id = 'chatbot-widget-container';
  document.body.appendChild(container);

  // Dynamically load React if not already available
  if (typeof React === 'undefined' || typeof ReactDOM === 'undefined') {
    console.error('React and ReactDOM must be loaded before initializing the chat widget');
    return;
  }

  // Render the chat widget
  const renderWidget = () => {
    // Apply theme
    const theme = finalConfig.theme === 'auto'
      ? (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light')
      : finalConfig.theme;

    document.body.setAttribute('data-chat-theme', theme);

    // Render the component
    ReactDOM.render(
      React.createElement(ChatWidget, { backendUrl: finalConfig.backendUrl }),
      container
    );
  };

  // Apply theme when system theme changes
  if (window.matchMedia && finalConfig.theme === 'auto') {
    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
    mediaQuery.addListener(() => {
      const theme = mediaQuery.matches ? 'dark' : 'light';
      document.body.setAttribute('data-chat-theme', theme);
    });
  }

  renderWidget();
};

// Export for use in module systems
export default ChatWidget;