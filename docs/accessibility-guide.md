# Accessibility Guide for RAG Chatbot

This document outlines the accessibility features implemented in the RAG Chatbot system and provides guidelines for maintaining accessibility compliance.

## Table of Contents
- [Overview](#overview)
- [Accessibility Standards](#accessibility-standards)
- [Implemented Features](#implemented-features)
- [Keyboard Navigation](#keyboard-navigation)
- [Screen Reader Support](#screen-reader-support)
- [Color and Contrast](#color-and-contrast)
- [Testing Guidelines](#testing-guidelines)
- [Development Best Practices](#development-best-practices)

## Overview

The RAG Chatbot system has been designed with accessibility in mind to ensure it can be used by people with various disabilities. This includes support for keyboard navigation, screen readers, and other assistive technologies.

## Accessibility Standards

The implementation follows the Web Content Accessibility Guidelines (WCAG) 2.1 AA standards:

- **Perceivable**: Information and UI components must be presentable in ways that users can perceive
- **Operable**: Interface components must be operable by all users
- **Understandable**: Information and UI operation must be understandable
- **Robust**: Content must be robust enough to work with various assistive technologies

## Implemented Features

### ARIA Labels and Roles

The chatbot includes appropriate ARIA attributes:

- **ChatWidget**:
  - `role="region"` with `aria-label="Chatbot widget"`
  - Dynamic `aria-label` for open/close button

- **ChatModal**:
  - `role="dialog"` with `aria-modal="true"`
  - `aria-labelledby` pointing to modal title
  - `aria-live` regions for dynamic content updates

- **Messages**:
  - `role="logitem"` for individual messages
  - `aria-label` to identify user vs bot messages

### Keyboard Navigation

- **Open/Close**: The chat widget button is fully keyboard accessible
- **Escape Key**: Closes the modal when focused
- **Tab Navigation**: Logical tab order through all interactive elements
- **Enter Key**: Submit messages in the input area

### Focus Management

- Proper focus management when modal opens (focuses on input)
- Focus returns to the open button when modal closes
- Focus is maintained within the modal when open
- Visual focus indicators for keyboard navigation

## Keyboard Navigation

### Chat Widget
- `Tab`: Navigate to the chat button
- `Space` or `Enter`: Open the chat modal

### Chat Modal
- `Escape`: Close the modal
- `Tab`: Navigate between input field and send button
- `Enter`: Submit message (when not holding Shift)
- `Shift + Enter`: Add line break in text area

### Message History
- Messages are read by screen readers as they appear
- Interactive elements within messages (source details) are keyboard accessible

## Screen Reader Support

### Announcements
- Loading states are announced: "Loading chat..."
- Error messages are announced immediately with `aria-live="assertive"`
- New messages are announced with `aria-live="polite"`

### Context
- Each message has appropriate labeling for screen readers
- Source details are announced when expanded
- Confidence indicators are announced for bot responses

### Interactive Elements
- Buttons have clear labels (e.g., "Open chatbot", "Close chat")
- Input areas have descriptive labels
- Dynamic content changes are announced appropriately

## Color and Contrast

The chatbot follows WCAG 2.1 AA contrast requirements:

- Normal text: Minimum 4.5:1 contrast ratio
- Large text: Minimum 3:1 contrast ratio
- User interface components: Minimum 3:1 contrast ratio

The design supports both light and dark themes with appropriate contrast in both modes.

## Testing Guidelines

### Automated Testing
Regular accessibility testing should include:

- WAVE (Web Accessibility Evaluation Tool)
- axe-core automated testing
- Lighthouse accessibility audits

### Manual Testing
- Keyboard navigation testing (Tab, Shift+Tab, Enter, Escape)
- Screen reader testing (NVDA, JAWS, VoiceOver)
- Focus order verification
- Color contrast verification

### Assistive Technology Testing
- Screen readers (NVDA, JAWS, VoiceOver)
- Keyboard-only navigation
- High contrast mode
- Screen magnification

## Development Best Practices

### HTML Structure
- Use semantic HTML elements where appropriate
- Maintain proper heading hierarchy
- Ensure all interactive elements are keyboard accessible
- Use ARIA attributes to enhance accessibility when needed

### Component Accessibility
Each component should implement:

1. **ChatWidget**:
   ```jsx
   <div role="region" aria-label="Chatbot widget">
     <button
       aria-label={isOpen ? "Close chatbot" : "Open chatbot"}
       aria-expanded={isOpen}
       aria-controls="chatbot-modal"
     >
   ```

2. **ChatModal**:
   ```jsx
   <div
     role="dialog"
     aria-modal="true"
     aria-labelledby="chat-modal-title"
     tabIndex={-1}
   >
   ```

3. **Messages**:
   ```jsx
   <div
     role="logitem"
     aria-label={isUser ? "User message" : "Assistant response"}
   >
   ```

### Focus Management
- Ensure focus is properly managed when components appear/disappear
- Return focus to appropriate elements after actions
- Use focus-trapping for modal dialogs
- Provide visible focus indicators

### ARIA Attributes
- Use `aria-live` for dynamic content updates
- Use `aria-describedby` and `aria-labelledby` for associations
- Use `role="status"` and `role="alert"` appropriately
- Use `aria-hidden="true"` for purely visual elements

## Common Accessibility Issues to Avoid

- Missing alternative text for visual elements
- Inadequate color contrast
- Non-keyboard accessible interactive elements
- Missing focus indicators
- Improper heading structure
- Dynamic content that isn't announced to screen readers
- Modals that don't trap focus

## Conclusion

Accessibility is an ongoing effort that requires continuous attention. All new features should be evaluated for accessibility impact, and regular testing should be performed to ensure the chatbot remains accessible to all users.