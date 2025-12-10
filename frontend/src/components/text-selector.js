/**
 * Text Selection Handler for RAG Chatbot
 * Handles text selection events and provides interface for selected-text RAG
 */

class TextSelector {
  constructor(options = {}) {
    this.options = {
      apiEndpoint: options.apiEndpoint || '/api/rag/selected',
      widget: options.widget || null, // Reference to the chat widget
      ...options
    };

    this.selectionTimeout = null;
    this.init();
  }

  init() {
    // Add event listeners for text selection
    document.addEventListener('mouseup', this.handleTextSelection.bind(this));
    document.addEventListener('keyup', (e) => {
      if (e.key === 'Escape') {
        this.hideSelectionButton();
      }
    });

    console.log('TextSelector: Initialized');
  }

  handleTextSelection() {
    // Clear any existing timeout
    if (this.selectionTimeout) {
      clearTimeout(this.selectionTimeout);
    }

    // Use a small timeout to ensure selection is complete
    this.selectionTimeout = setTimeout(() => {
      const selectedText = window.getSelection().toString().trim();

      if (selectedText && this.isValidSelection(selectedText)) {
        this.showSelectionButton(selectedText);
      } else {
        this.hideSelectionButton();
      }
    }, 100);
  }

  isValidSelection(text) {
    // Check if the selection is meaningful (not just whitespace, not too short, not too long)
    return text.length > 10 && text.length < 2000;
  }

  showSelectionButton(selectedText) {
    // Remove any existing button
    this.hideSelectionButton();

    // Create the button element
    const button = document.createElement('button');
    button.id = 'text-selector-ask-button';
    button.className = 'text-selector-button';
    button.textContent = 'Ask Chatbot';
    button.setAttribute('aria-label', 'Ask chatbot about selected text');

    // Position the button near the selection
    const selectionRange = window.getSelection().getRangeAt(0);
    const rect = selectionRange.getBoundingClientRect();

    // Position above the selection with some offset
    button.style.position = 'fixed';
    button.style.top = (rect.top + window.scrollY - 40) + 'px';
    button.style.left = (rect.left + window.scrollX) + 'px';
    button.style.zIndex = '10000';
    button.style.background = '#4f46e5';
    button.style.color = 'white';
    button.style.border = 'none';
    button.style.borderRadius = '4px';
    button.style.padding = '6px 12px';
    button.style.cursor = 'pointer';
    button.style.fontSize = '14px';
    button.style.boxShadow = '0 2px 6px rgba(0,0,0,0.15)';

    // Add the button to the document
    document.body.appendChild(button);

    // Add click event to button
    button.addEventListener('click', () => {
      this.askAboutSelectedText(selectedText);
      this.hideSelectionButton();
    });
  }

  hideSelectionButton() {
    const existingButton = document.getElementById('text-selector-ask-button');
    if (existingButton) {
      existingButton.remove();
    }
  }

  async askAboutSelectedText(selectedText) {
    console.log('Asking about selected text:', selectedText.substring(0, 50) + '...');

    // If we have a reference to the chat widget, use it to open the chat
    if (this.options.widget) {
      // Add the selected text as context in the chat
      this.options.widget.addMessageToUI(`I'd like to ask about this selected text: "${selectedText.substring(0, 50)}..."`, 'user');

      // Call the selected text RAG endpoint
      try {
        const response = await fetch(this.options.apiEndpoint, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            selected_text: selectedText,
            query: "Please explain or provide information about this selected text."
          })
        });

        if (!response.ok) {
          throw new Error(`Selected text API call failed with status ${response.status}`);
        }

        const data = await response.json();

        // Add the response to the chat widget
        this.options.widget.addMessageToUI(data.answer || "I couldn't process that selected text. Please try again.", 'bot');
      } catch (error) {
        console.error('Error with selected text query:', error);
        this.options.widget.addMessageToUI('Chatbot is unavailable. Please try again later.', 'bot', true);
      }
    } else {
      // If no widget reference, we might want to open the chat in another way
      // For now, we'll just log the action
      console.log('No widget reference provided, would normally open chat with selected text');
    }
  }

  // Method to update options dynamically
  updateOptions(newOptions) {
    this.options = { ...this.options, ...newOptions };
  }
}

// Make TextSelector available globally
window.TextSelector = TextSelector;