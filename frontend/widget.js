/**
 * Standalone Chatbot Widget for Book Integration
 * This file can be included in any HTML page to add the chatbot functionality
 */

(function (global, factory) {
  typeof exports === 'object' && typeof module !== 'undefined' ? factory() :
  typeof define === 'function' && define.amd ? define(factory) :
  (factory());
}(this, (function () {
  'use strict';

  // Visual indicator for selected text
  let visualIndicator = null;

  function showVisualIndicator(rect) {
    // Remove existing indicator if present
    removeVisualIndicator();

    // Create a visual indicator element
    visualIndicator = document.createElement('div');
    visualIndicator.id = 'chatbot-selection-indicator';
    visualIndicator.style.position = 'absolute';
    visualIndicator.style.left = (rect.right - 20) + 'px';
    visualIndicator.style.top = (rect.top - 20) + 'px';
    visualIndicator.style.width = '24px';
    visualIndicator.style.height = '24px';
    visualIndicator.style.backgroundColor = '#4f46e5';
    visualIndicator.style.borderRadius = '50%';
    visualIndicator.style.display = 'flex';
    visualIndicator.style.alignItems = 'center';
    visualIndicator.style.justifyContent = 'center';
    visualIndicator.style.color = 'white';
    visualIndicator.style.fontSize = '12px';
    visualIndicator.style.fontWeight = 'bold';
    visualIndicator.style.zIndex = '10000';
    visualIndicator.style.cursor = 'pointer';
    visualIndicator.style.boxShadow = '0 2px 4px rgba(0,0,0,0.2)';
    visualIndicator.style.border = '2px solid white';
    visualIndicator.innerHTML = '💬';

    // Add click handler to open chat with selected text
    visualIndicator.addEventListener('click', function() {
      // Trigger a custom event that the chat widget can listen for
      const event = new CustomEvent('chatbotTextSelected', {
        detail: {
          selectedText: window.getSelection().toString().trim()
        }
      });
      document.dispatchEvent(event);

      // Also open the chat widget if it's not already open
      const floatButton = document.querySelector('.chatbot-float-button');
      if (floatButton) {
        floatButton.click();
      }
    });

    document.body.appendChild(visualIndicator);
  }

  function removeVisualIndicator() {
    if (visualIndicator && document.body.contains(visualIndicator)) {
      document.body.removeChild(visualIndicator);
      visualIndicator = null;
    }
  }

  function getTextSelection() {
    const selection = window.getSelection();
    if (selection.rangeCount > 0) {
      const range = selection.getRangeAt(0);
      const selectedText = selection.toString().trim();

      if (selectedText) {
        // Get the bounding rectangle of the selection for positioning
        const rect = range.getBoundingClientRect();

        return {
          text: selectedText,
          range: range,
          rect: rect,
          startContainer: range.startContainer,
          startOffset: range.startOffset,
          endContainer: range.endContainer,
          endOffset: range.endOffset
        };
      }
    }

    return null;
  }

  function addTextSelectionListenerWithVisualFeedback(callback) {
    const handleSelection = function() {
      const selection = getTextSelection();

      // Remove previous indicator
      removeVisualIndicator();

      if (selection) {
        // Show visual indicator near the selected text
        showVisualIndicator(selection.rect);

        // Call the original callback
        if (typeof callback === 'function') {
          callback(selection);
        }
      }
    };

    // Listen for mouseup event (when user finishes selecting text)
    document.addEventListener('mouseup', handleSelection);

    // Also listen for keyup with shift key (for keyboard text selection)
    document.addEventListener('keyup', function(e) {
      if (e.key === 'Shift') {
        setTimeout(function() {
          const selection = getTextSelection();

          // Remove previous indicator
          removeVisualIndicator();

          if (selection) {
            // Show visual indicator near the selected text
            showVisualIndicator(selection.rect);

            // Call the original callback
            if (typeof callback === 'function') {
              callback(selection);
            }
          }
        }, 0);
      }
    });

    // Return a function to remove the listener
    return function() {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', function(e) {
        if (e.key === 'Shift') {
          setTimeout(function() {
            const selection = getTextSelection();

            // Remove previous indicator
            removeVisualIndicator();

            if (selection) {
              // Show visual indicator near the selected text
              showVisualIndicator(selection.rect);

              // Call the original callback
              if (typeof callback === 'function') {
                callback(selection);
              }
            }
          }, 0);
        }
      });
    };
  }

  // Widget persistence functions
  var WIDGET_STATE_KEY = 'chatbot_widget_state';

  function saveWidgetState(state) {
    try {
      var widgetState = {
        ...state,
        timestamp: Date.now() // Add timestamp for potential expiration
      };
      sessionStorage.setItem(WIDGET_STATE_KEY, JSON.stringify(widgetState));
    } catch (error) {
      console.warn('Could not save widget state to sessionStorage:', error);
    }
  }

  function loadWidgetState() {
    try {
      var savedState = sessionStorage.getItem(WIDGET_STATE_KEY);
      if (savedState) {
        var parsedState = JSON.parse(savedState);

        // Check if state is expired (older than 1 hour)
        var now = Date.now();
        var oneHour = 60 * 60 * 1000;

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
  }

  function setupNavigationHandler(onNavigate) {
    // Handle browser back/forward buttons
    var handlePopState = function() {
      onNavigate();
    };

    // Handle hash changes (common in SPAs)
    var handleHashChange = function() {
      onNavigate();
    };

    // Handle custom route changes (for React/Vue apps)
    var handleCustomRouteChange = function() {
      onNavigate();
    };

    // Handle page unload to save state
    var handleBeforeUnload = function() {
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
    return function() {
      window.removeEventListener('popstate', handlePopState);
      window.removeEventListener('hashchange', handleHashChange);
      window.removeEventListener('routeChange', handleCustomRouteChange);
      window.removeEventListener('beforeunload', handleBeforeUnload);
      window.removeEventListener('turbolinks:before-visit', handleBeforeUnload);
      window.removeEventListener('turbolinks:load', handlePopState);
      window.removeEventListener('historyPush', handlePopState);
      window.removeEventListener('historyReplace', handlePopState);
    };
  }

  // Check if React and ReactDOM are available
  if (typeof React === 'undefined' || typeof ReactDOM === 'undefined') {
    console.error('React and ReactDOM must be loaded before the chatbot widget');
    console.error('Please include React and ReactDOM before this script');
    return;
  }

  // ChatWidget component implementation
  const ChatWidget = (function() {
    // Simple implementation for the widget without requiring JSX transformation
    function createWidget(config = {}) {
      // Default configuration - use environment variable or fallback
      const defaultConfig = {
        backendUrl: (typeof window !== 'undefined' && window.env?.NEXT_PUBLIC_RAG_BACKEND_URL) || 'http://localhost:8000',
        theme: 'auto'
      };

      const finalConfig = Object.assign(defaultConfig, config);

      // Load saved state if available
      const savedState = loadWidgetState();
      let isOpen = (savedState && savedState.isOpen) || false;

      // Create the widget container
      const container = document.createElement('div');
      container.id = 'chatbot-widget-container';
      document.body.appendChild(container);

      // Apply theme
      const theme = finalConfig.theme === 'auto'
        ? (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light')
        : finalConfig.theme;

      document.body.setAttribute('data-chat-theme', theme);

      // Add text selection listener with visual feedback
      const removeTextSelectionListener = addTextSelectionListenerWithVisualFeedback(function(selection) {
        // This callback can be used to handle text selection if needed
        // For now, we're just using the visual feedback and custom event
      });

      // Handle custom event when user clicks the visual indicator
      function handleCustomTextSelection(event) {
        // This will be handled when the modal is created
      }

      // Add event listener for the custom event
      document.addEventListener('chatbotTextSelected', handleCustomTextSelection);

      // Create the floating button
      const floatButton = document.createElement('button');
      floatButton.className = 'chatbot-float-button';
      floatButton.setAttribute('aria-label', isOpen ? 'Close chatbot' : 'Open chatbot');
      floatButton.innerHTML = '<div class="chatbot-icon">💬</div>';

      // Add event listener to the button
      const modal = createModal(finalConfig.backendUrl);

      floatButton.addEventListener('click', function() {
        if (!isOpen) {
          document.body.appendChild(modal);
          isOpen = true;
          floatButton.setAttribute('aria-label', 'Close chatbot');
        } else {
          if (modal.parentNode) {
            modal.parentNode.removeChild(modal);
          }
          isOpen = false;
          floatButton.setAttribute('aria-label', 'Open chatbot');
        }

        // Save state after toggle
        saveWidgetState({
          isOpen: isOpen,
          timestamp: Date.now()
        });
      });

      document.body.appendChild(floatButton);

      // Handle theme changes
      if (window.matchMedia && finalConfig.theme === 'auto') {
        const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
        mediaQuery.addListener(function() {
          const theme = mediaQuery.matches ? 'dark' : 'light';
          document.body.setAttribute('data-chat-theme', theme);
        });
      }

      // Set up navigation handlers for persistence
      const cleanupNavigation = setupNavigationHandler(function() {
        // On navigation, save the current state
        saveWidgetState({
          isOpen: isOpen,
          timestamp: Date.now()
        });
      });

      // Store the cleanup function to be used later
      window.chatbotCleanup = function() {
        removeTextSelectionListener();
        document.removeEventListener('chatbotTextSelected', handleCustomTextSelection);
        cleanupNavigation(); // Clean up navigation handlers
      };
    }

    function createModal(backendUrl) {
      const modal = document.createElement('div');
      modal.className = 'chatbot-modal-overlay';
      modal.style.cssText = `
        position: fixed;
        top: 0;
        left: 0;
        right: 0;
        bottom: 0;
        background: rgba(0, 0, 0, 0.5);
        display: flex;
        align-items: center;
        justify-content: center;
        z-index: 10001;
        padding: 20px;
      `;

      modal.innerHTML = `
        <div class="chatbot-modal" style="
          width: 100%;
          max-width: 500px;
          height: 70vh;
          max-height: 600px;
          background: white;
          border-radius: 12px;
          display: flex;
          flex-direction: column;
          box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
          overflow: hidden;
        ">
          <div class="chatbot-header" style="
            padding: 16px 20px;
            background: #4f46e5;
            color: white;
            display: flex;
            justify-content: space-between;
            align-items: center;
          ">
            <h3 style="margin: 0; font-size: 18px;">Book Assistant</h3>
            <button class="chatbot-close-button" style="
              background: none;
              border: none;
              color: white;
              font-size: 24px;
              cursor: pointer;
              padding: 0;
              width: 30px;
              height: 30px;
              display: flex;
              align-items: center;
              justify-content: center;
            " aria-label="Close chat">×</button>
          </div>
          <div class="chatbot-messages" style="
            flex: 1;
            overflow-y: auto;
            padding: 20px;
            display: flex;
            flex-direction: column;
            gap: 16px;
            background: #f9fafb;
          ">
            <div class="chatbot-message bot-message" style="
              display: flex;
              max-width: 85%;
            ">
              <div class="chatbot-message-content" style="
                background: white;
                color: #374151;
                border-radius: 4px 18px 18px 18px;
                box-shadow: 0 1px 2px rgba(0, 0, 0, 0.05);
                padding: 12px 16px;
                position: relative;
                min-width: 60px;
                max-width: 100%;
              ">
                <div class="message-text">
                  Hello! I'm your book assistant. Ask me anything about the book content.
                </div>
              </div>
            </div>
          </div>
          <div class="chatbot-input-form" style="
            padding: 16px 20px;
            background: white;
            border-top: 1px solid #e5e7eb;
          ">
            <div class="chatbot-input-container" style="
              display: flex;
              gap: 8px;
              align-items: flex-end;
            ">
              <textarea
                class="chatbot-textarea"
                placeholder="Ask a question about the book..."
                style="
                  flex: 1;
                  border: 1px solid #d1d5db;
                  border-radius: 18px;
                  padding: 12px 16px;
                  resize: none;
                  max-height: 100px;
                  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
                  font-size: 14px;
                  outline: none;
                "
              ></textarea>
              <button
                class="chatbot-send-button"
                style="
                  background: #4f46e5;
                  color: white;
                  border: none;
                  border-radius: 18px;
                  padding: 12px 20px;
                  cursor: pointer;
                  font-size: 14px;
                  font-weight: 500;
                "
              >
                Send
              </button>
            </div>
          </div>
        </div>
      `;

      // Handle custom event when user clicks the visual indicator
      function handleCustomTextSelectionForModal(event) {
        const selectedText = event.detail.selectedText;
        if (selectedText && selectedText.length > 0) {
          // Get the textarea and set the selected text as context
          const textarea = modal.querySelector('.chatbot-textarea');
          if (textarea) {
            // Pre-fill the textarea with a prompt about the selected text
            textarea.value = `About this text: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}". `;
            textarea.focus();

            // Move cursor to the end of the pre-filled text
            textarea.setSelectionRange(textarea.value.length, textarea.value.length);
          }
        }
      }

      // Add event listener for the custom event
      document.addEventListener('chatbotTextSelected', handleCustomTextSelectionForModal);

      // Add close functionality
      const closeButton = modal.querySelector('.chatbot-close-button');
      closeButton.addEventListener('click', function() {
        // Remove the event listener when modal is closed
        document.removeEventListener('chatbotTextSelected', handleCustomTextSelectionForModal);
        if (modal.parentNode) {
          modal.parentNode.removeChild(modal);
        }
      });

      // Add overlay close functionality
      modal.addEventListener('click', function(e) {
        if (e.target === modal) {
          // Remove the event listener when modal is closed
          document.removeEventListener('chatbotTextSelected', handleCustomTextSelectionForModal);
          if (modal.parentNode) {
            modal.parentNode.removeChild(modal);
          }
        }
      });

      return modal;
    }

    return {
      init: createWidget
    };
  })();

  // Initialize the chatbot widget
  global.ChatbotWidget = {
    init: function(config) {
      if (typeof ChatWidget !== 'undefined' && typeof ChatWidget.init === 'function') {
        ChatWidget.init(config);
      } else {
        console.error('ChatWidget component not found or init function not available');
      }
    }
  };

  // Auto-initialize if configured to do so
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function() {
      // Check for data attributes to auto-initialize
      const widgetElement = document.querySelector('[data-chatbot-widget]');
      if (widgetElement) {
        const config = {
          backendUrl: widgetElement.getAttribute('data-backend-url') || (typeof window !== 'undefined' && window.env?.NEXT_PUBLIC_RAG_BACKEND_URL) || 'http://localhost:8000',
          theme: widgetElement.getAttribute('data-theme') || 'auto'
        };
        global.ChatbotWidget.init(config);
      }
    });
  } else {
    // DOM is already loaded
    const widgetElement = document.querySelector('[data-chatbot-widget]');
    if (widgetElement) {
      const config = {
        backendUrl: widgetElement.getAttribute('data-backend-url') || 'https://fastapi-backend-for-book.vercel.app',
        theme: widgetElement.getAttribute('data-theme') || 'auto'
      };
      global.ChatbotWidget.init(config);
    }
  }

})));