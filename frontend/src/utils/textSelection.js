/**
 * Utility functions for handling text selection in the browser
 */

export const getTextSelection = () => {
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
};

/**
 * Function to get selected text specifically for the chatbot
 */
export const getSelectedTextForChat = () => {
  const selection = window.getSelection();
  if (selection.rangeCount > 0) {
    const selectedText = selection.toString().trim();
    if (selectedText && selectedText.length > 0) {
      // Limit the selected text to a reasonable length
      return selectedText.length > 2000 ? selectedText.substring(0, 2000) + '...' : selectedText;
    }
  }
  return null;
};

export const addTextSelectionListener = (callback) => {
  const handleSelection = () => {
    const selection = getTextSelection();
    if (selection) {
      callback(selection);
    }
  };

  // Listen for mouseup event (when user finishes selecting text)
  document.addEventListener('mouseup', handleSelection);

  // Also listen for keyup with shift key (for keyboard text selection)
  document.addEventListener('keyup', (e) => {
    if (e.key === 'Shift') {
      setTimeout(() => {
        const selection = getTextSelection();
        if (selection) {
          callback(selection);
        }
      }, 0);
    }
  });

  // Return a function to remove the listener
  return () => {
    document.removeEventListener('mouseup', handleSelection);
    document.removeEventListener('keyup', (e) => {
      if (e.key === 'Shift') {
        setTimeout(() => {
          const selection = getTextSelection();
          if (selection) {
            callback(selection);
          }
        }, 0);
      }
    });
  };
};

export const highlightText = (range) => {
  if (!range) return null;

  // Create a temporary element to wrap the selected text
  const highlight = document.createElement('span');
  highlight.style.backgroundColor = 'yellow';
  highlight.style.opacity = '0.5';

  try {
    range.surroundContents(highlight);
    return highlight;
  } catch (error) {
    // If surroundContents fails (e.g., if the range crosses element boundaries),
    // we'll create a new range
    const contents = range.extractContents();
    highlight.appendChild(contents);
    range.insertNode(highlight);
    return highlight;
  }
};

// Visual indicator for selected text
let visualIndicator = null;

export const showVisualIndicator = (rect) => {
  // Remove existing indicator if present
  removeVisualIndicator();

  // Create a visual indicator element
  visualIndicator = document.createElement('div');
  visualIndicator.id = 'chatbot-selection-indicator';
  visualIndicator.style.position = 'absolute';
  visualIndicator.style.left = rect.right - 20 + 'px';
  visualIndicator.style.top = rect.top - 20 + 'px';
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
  visualIndicator.addEventListener('click', () => {
    // Trigger a custom event that the chat widget can listen for
    const event = new CustomEvent('chatbotTextSelected', {
      detail: {
        selectedText: window.getSelection().toString().trim()
      }
    });
    document.dispatchEvent(event);
  });

  document.body.appendChild(visualIndicator);
};

export const removeVisualIndicator = () => {
  if (visualIndicator && document.body.contains(visualIndicator)) {
    document.body.removeChild(visualIndicator);
    visualIndicator = null;
  }
};

export const removeHighlight = (highlightElement) => {
  if (!highlightElement || !highlightElement.parentNode) return;

  const parent = highlightElement.parentNode;
  const contents = highlightElement.childNodes;

  // Move the contents back to the parent
  while (contents.length > 0) {
    parent.insertBefore(contents[0], highlightElement);
  }

  // Remove the highlight element
  parent.removeChild(highlightElement);
};

// Enhanced text selection listener with visual feedback
export const addTextSelectionListenerWithVisualFeedback = (callback) => {
  const handleSelection = () => {
    const selection = getTextSelection();

    // Remove previous indicator
    removeVisualIndicator();

    if (selection) {
      // Show visual indicator near the selected text
      showVisualIndicator(selection.rect);

      // Call the original callback
      callback(selection);
    }
  };

  // Listen for mouseup event (when user finishes selecting text)
  document.addEventListener('mouseup', handleSelection);

  // Also listen for keyup with shift key (for keyboard text selection)
  document.addEventListener('keyup', (e) => {
    if (e.key === 'Shift') {
      setTimeout(() => {
        const selection = getTextSelection();

        // Remove previous indicator
        removeVisualIndicator();

        if (selection) {
          // Show visual indicator near the selected text
          showVisualIndicator(selection.rect);

          // Call the original callback
          callback(selection);
        }
      }, 0);
    }
  });

  // Return a function to remove the listener
  return () => {
    document.removeEventListener('mouseup', handleSelection);
    document.removeEventListener('keyup', (e) => {
      if (e.key === 'Shift') {
        setTimeout(() => {
          const selection = getTextSelection();

          // Remove previous indicator
          removeVisualIndicator();

          if (selection) {
            // Show visual indicator near the selected text
            showVisualIndicator(selection.rect);

            // Call the original callback
            callback(selection);
          }
        }, 0);
      }
    });
  };
};