# RAG Chatbot Integration Guide

This guide explains how to integrate the RAG Chatbot widget into your Docusaurus-based book website.

## Quick Integration

### Method 1: Script Tag (Recommended)

Add the following script tag to your HTML pages:

```html
<script src="path/to/chatbot-widget.js"></script>
<script>
  // Initialize the chatbot with your backend URL
  window.ChatbotWidget.init({
    backendUrl: 'https://your-backend-url.com',
    theme: 'auto' // 'light', 'dark', or 'auto' (default: 'auto')
  });
</script>
```

### Method 2: Auto-initialize with Data Attributes

Add data attributes to any HTML element on your page:

```html
<div data-chatbot-widget
     data-backend-url="https://your-backend-url.com"
     data-theme="auto">
</div>
```

The chatbot will automatically initialize when the page loads.

## Docusaurus Integration

### Option 1: Using Docusaurus Config

Add the script to your `docusaurus.config.js`:

```js
module.exports = {
  // ... other config
  scripts: [
    {
      src: '/js/chatbot-widget.js',
      async: true,
    }
  ],
  // ... rest of config
};
```

Then initialize in your page:

```html
<script>
  window.ChatbotWidget.init({
    backendUrl: 'https://your-backend-url.com'
  });
</script>
```

### Option 2: Custom React Component

If you're using React with Docusaurus, you can import and use the component directly:

```jsx
import { useEffect } from 'react';

export default function ChatbotIntegration() {
  useEffect(() => {
    if (typeof window !== 'undefined' && window.ChatbotWidget) {
      window.ChatbotWidget.init({
        backendUrl: 'https://your-backend-url.com',
        theme: 'auto'
      });
    }
  }, []);

  return null;
}
```

## Configuration Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `backendUrl` | string | `'http://localhost:8000'` | URL of your RAG Chatbot backend |
| `theme` | `'light' \| 'dark' \| 'auto'` | `'auto'` | Theme for the chatbot UI |

## Styling

The chatbot comes with default styles, but you can customize them by overriding CSS classes:

- `.chatbot-container` - The main container
- `.chatbot-float-button` - The floating action button
- `.chatbot-modal` - The chat modal
- `.chatbot-message` - Individual messages
- `.chatbot-input-form` - The input area

## Troubleshooting

### Chatbot Not Appearing

1. Ensure the script is loaded after React and ReactDOM
2. Check that your backend URL is correct
3. Verify that your backend is running and accessible

### CORS Issues

Make sure your backend allows requests from your website's domain.

### Performance

The widget is designed to be lightweight, but if you notice performance issues:
1. Check your network connection to the backend
2. Verify that your backend can handle requests efficiently
3. Consider implementing caching for frequent queries

## Security

- Never expose your backend API keys in client-side code
- Use HTTPS for production deployments
- Implement rate limiting on your backend