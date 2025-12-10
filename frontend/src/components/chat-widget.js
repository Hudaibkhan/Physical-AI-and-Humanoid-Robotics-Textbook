/**
 * ChatKit Widget for RAG Chatbot Integration
 * Provides a floating chat widget that connects to the RAG backend
 */

class ChatWidget {
  constructor(options = {}) {
    this.options = {
      apiEndpoint: options.apiEndpoint || '/api/conversation',
      agentId: options.agentId || null,
      containerId: options.containerId || 'chatkit-container',
      position: options.position || 'bottom-right', // bottom-left, bottom-right, top-left, top-right
      title: options.title || 'AI Assistant',
      welcomeMessage: options.welcomeMessage || 'Hello! How can I help you today?',
      fallbackMessage: options.fallbackMessage || 'Chatbot is unavailable. Please try again later.',
      ...options
    };

    this.isOpen = false;
    this.messages = [];
    this.initializeWidget();
  }

  initializeWidget() {
    // Create the widget HTML structure
    this.createWidgetStructure();

    // Attach event listeners
    this.attachEventListeners();

    // Try to initialize the connection
    this.initializeConnection();
  }

  createWidgetStructure() {
    // Create main container
    const container = document.createElement('div');
    container.id = this.options.containerId;
    container.className = 'chatkit-container';
    container.innerHTML = `
      <div class="chatkit-button" id="chatkit-toggle-button" aria-label="Open chatbot">
        <span class="chatkit-icon">💬</span>
      </div>
      <div class="chatkit-widget" id="chatkit-widget" style="display: none;">
        <div class="chatkit-header">
          <div class="chatkit-title">${this.options.title}</div>
          <button class="chatkit-close" id="chatkit-close-button" aria-label="Close chat">&times;</button>
        </div>
        <div class="chatkit-messages" id="chatkit-messages">
          <div class="chatkit-message chatkit-message-bot">
            ${this.options.welcomeMessage}
          </div>
        </div>
        <div class="chatkit-input-area">
          <input
            type="text"
            class="chatkit-input"
            id="chatkit-user-input"
            placeholder="Type your message..."
            aria-label="Type your message"
          />
          <button class="chatkit-send" id="chatkit-send-button" aria-label="Send message">
            <span class="chatkit-send-icon">➤</span>
          </button>
        </div>
      </div>
    `;

    // Add the container to the body
    document.body.appendChild(container);

    // Apply positioning styles based on options
    this.applyPositioning();
  }

  applyPositioning() {
    const widget = document.getElementById('chatkit-widget');
    const button = document.getElementById('chatkit-toggle-button');

    // Set positioning based on options
    const positionClasses = this.options.position.split('-');
    const vertical = positionClasses[0]; // top or bottom
    const horizontal = positionClasses[1]; // left or right

    // Apply position to button
    button.style.position = 'fixed';
    button.style[vertical] = '20px';
    button.style[horizontal] = '20px';
    button.style.zIndex = '10000';

    // Apply position to widget
    widget.style.position = 'fixed';
    widget.style[vertical] = '20px';
    widget.style[horizontal] = '20px';
    widget.style.zIndex = '9999';
  }

  attachEventListeners() {
    const toggleButton = document.getElementById('chatkit-toggle-button');
    const closeButton = document.getElementById('chatkit-close-button');
    const userInput = document.getElementById('chatkit-user-input');
    const sendButton = document.getElementById('chatkit-send-button');
    const messagesContainer = document.getElementById('chatkit-messages');

    // Toggle chat window
    toggleButton.addEventListener('click', () => {
      this.toggleChat();
    });

    // Close chat window
    closeButton.addEventListener('click', () => {
      this.closeChat();
    });

    // Send message on button click
    sendButton.addEventListener('click', () => {
      this.sendMessage();
    });

    // Send message on Enter key
    userInput.addEventListener('keypress', (e) => {
      if (e.key === 'Enter') {
        this.sendMessage();
      }
    });

    // Add text selection listener for selected text RAG
    document.addEventListener('mouseup', () => {
      setTimeout(() => {
        this.handleTextSelection();
      }, 100); // Small delay to ensure selection is complete
    });
  }

  toggleChat() {
    const widget = document.getElementById('chatkit-widget');
    if (this.isOpen) {
      this.closeChat();
    } else {
      this.openChat();
    }
  }

  openChat() {
    const widget = document.getElementById('chatkit-widget');
    widget.style.display = 'flex';
    widget.style.flexDirection = 'column';
    this.isOpen = true;

    // Focus on input when opening
    const userInput = document.getElementById('chatkit-user-input');
    setTimeout(() => userInput.focus(), 100);
  }

  closeChat() {
    const widget = document.getElementById('chatkit-widget');
    widget.style.display = 'none';
    this.isOpen = false;
  }

  async sendMessage() {
    const userInput = document.getElementById('chatkit-user-input');
    const message = userInput.value.trim();

    if (!message) return;

    // Add user message to UI
    this.addMessageToUI(message, 'user');
    userInput.value = '';

    try {
      // Call the backend API
      const response = await this.callBackendAPI(message);

      // Add bot response to UI
      this.addMessageToUI(response.answer || response, 'bot');
    } catch (error) {
      console.error('Error sending message:', error);
      this.addMessageToUI(this.options.fallbackMessage, 'bot', true);
    }
  }

  async callBackendAPI(message, session_id = null, context = null) {
    // For now, we'll make a simple fetch call
    // In a real implementation, this would connect to your backend
    const requestBody = {
      message: message,
      session_id: session_id || this.sessionId,
      context: context || null
    };

    const response = await fetch(this.options.apiEndpoint, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody)
    });

    if (!response.ok) {
      throw new Error(`API call failed with status ${response.status}`);
    }

    return await response.json();
  }

  addMessageToUI(message, sender, isError = false) {
    const messagesContainer = document.getElementById('chatkit-messages');

    const messageElement = document.createElement('div');
    messageElement.className = `chatkit-message chatkit-message-${sender} ${isError ? 'chatkit-message-error' : ''}`;
    messageElement.textContent = message;

    messagesContainer.appendChild(messageElement);

    // Scroll to bottom
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  async initializeConnection() {
    // Check if backend is available
    try {
      const response = await fetch('/api/health');
      if (!response.ok) {
        throw new Error('Backend health check failed');
      }
      console.log('ChatWidget: Backend connection established');
    } catch (error) {
      console.error('ChatWidget: Backend connection failed', error);
      // Don't show error to user at initialization, only when trying to send messages
    }
  }

  handleTextSelection() {
    const selectedText = window.getSelection().toString().trim();

    if (selectedText && this.isValidSelection(selectedText)) { // Only show for meaningful selections
      // Create a temporary button to ask about selected text
      this.showSelectedTextButton(selectedText);
    }
  }

  isValidSelection(text) {
    // Check if the selection is meaningful (not just whitespace, not too short, not too long)
    return text.length > 10 && text.length < 2000;
  }

  showSelectedTextButton(selectedText) {
    // Remove any existing button
    this.hideSelectedTextButton();

    // Create the button element
    const button = document.createElement('button');
    button.id = 'chatkit-ask-button';
    button.className = 'chatkit-ask-selected-button';
    button.textContent = 'Ask Chatbot';
    button.setAttribute('aria-label', 'Ask chatbot about selected text');

    // Position the button near the selection
    const selectionRange = window.getSelection().getRangeAt(0);
    const rect = selectionRange.getBoundingClientRect();

    // Position above the selection with some offset
    button.style.position = 'fixed';
    button.style.top = (rect.top + window.scrollY - 40) + 'px';
    button.style.left = (rect.left + window.scrollX) + 'px';
    button.style.zIndex = '10001';
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
      this.hideSelectedTextButton();
    });
  }

  hideSelectedTextButton() {
    const existingButton = document.getElementById('chatkit-ask-button');
    if (existingButton) {
      existingButton.remove();
    }
  }

  async askAboutSelectedText(selectedText) {
    this.openChat();

    // Add user's intent to the chat
    this.addMessageToUI(`I'd like to ask about this selected text: "${selectedText.substring(0, 50)}..."`, 'user');

    try {
      // Call the selected text RAG endpoint
      const response = await fetch('/api/rag/selected', {
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
      this.addMessageToUI(data.answer || "I couldn't process that selected text. Please try again.", 'bot');
    } catch (error) {
      console.error('Error with selected text query:', error);
      this.addMessageToUI(this.options.fallbackMessage, 'bot', true);
    }
  }

  // Method to update widget options dynamically
  updateOptions(newOptions) {
    this.options = { ...this.options, ...newOptions };
  }
}

// CSS styles for the chat widget
const styles = `
  .chatkit-container {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
  }

  .chatkit-button {
    width: 60px;
    height: 60px;
    border-radius: 50%;
    background: #4f46e5;
    color: white;
    display: flex;
    align-items: center;
    justify-content: center;
    cursor: pointer;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
    transition: all 0.3s ease;
  }

  .chatkit-button:hover {
    background: #4338ca;
    transform: scale(1.05);
  }

  .chatkit-icon {
    font-size: 24px;
  }

  .chatkit-widget {
    width: 350px;
    height: 500px;
    background: white;
    border-radius: 8px;
    box-shadow: 0 10px 25px rgba(0, 0, 0, 0.15);
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }

  .chatkit-header {
    background: #4f46e5;
    color: white;
    padding: 16px;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  .chatkit-title {
    font-weight: 600;
  }

  .chatkit-close {
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
  }

  .chatkit-close:hover {
    background: rgba(255, 255, 255, 0.2);
    border-radius: 50%;
  }

  .chatkit-messages {
    flex: 1;
    padding: 16px;
    overflow-y: auto;
    display: flex;
    flex-direction: column;
    gap: 12px;
  }

  .chatkit-message {
    max-width: 80%;
    padding: 10px 14px;
    border-radius: 18px;
    font-size: 14px;
    line-height: 1.4;
  }

  .chatkit-message-user {
    align-self: flex-end;
    background: #4f46e5;
    color: white;
  }

  .chatkit-message-bot {
    align-self: flex-start;
    background: #f3f4f6;
    color: #374151;
  }

  .chatkit-message-error {
    background: #fee2e2;
    color: #b91c1c;
  }

  .chatkit-input-area {
    display: flex;
    padding: 16px;
    border-top: 1px solid #e5e7eb;
    background: white;
  }

  .chatkit-input {
    flex: 1;
    padding: 10px 14px;
    border: 1px solid #d1d5db;
    border-radius: 20px;
    outline: none;
  }

  .chatkit-input:focus {
    border-color: #4f46e5;
  }

  .chatkit-send {
    background: #4f46e5;
    color: white;
    border: none;
    border-radius: 50%;
    width: 40px;
    height: 40px;
    margin-left: 8px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .chatkit-send:hover {
    background: #4338ca;
  }

  .chatkit-send-icon {
    font-size: 16px;
  }

  .chatkit-ask-selected-button {
    position: absolute;
    background: #4f46e5;
    color: white;
    border: none;
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 12px;
    cursor: pointer;
    z-index: 10001;
    box-shadow: 0 2px 4px rgba(0,0,0,0.2);
  }
`;

// Add styles to the document
const styleSheet = document.createElement('style');
styleSheet.type = 'text/css';
styleSheet.innerText = styles;
document.head.appendChild(styleSheet);

// Make ChatWidget available globally
window.ChatWidget = ChatWidget;

// Auto-initialize if configuration is provided
document.addEventListener('DOMContentLoaded', function() {
  // Check for data attributes or global config
  const configElement = document.querySelector('[data-chatkit-config]');
  if (configElement) {
    const config = JSON.parse(configElement.getAttribute('data-chatkit-config'));
    new ChatWidget(config);
  }
});