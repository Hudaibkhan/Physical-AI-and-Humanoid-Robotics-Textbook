// server.js - Express server to handle Better Auth API routes alongside Docusaurus
const express = require('express');
const path = require('path');
const { auth } = require('./src/auth/auth.config');

// Create Express app
const app = express();
const port = process.env.PORT || 3002;

// Middleware to parse JSON
app.use(express.json());

// Serve static files from the Docusaurus build directory
app.use(express.static(path.join(__dirname, 'build')));

// Mount Better Auth API routes
app.use('/api/auth', auth.handler);

// For any other routes, serve the Docusaurus index.html
app.get('*', (req, res) => {
  res.sendFile(path.join(__dirname, 'build', 'index.html'));
});

app.listen(port, () => {
  console.log(`Server running at https://fastapi-backend-for-book.vercel.app`);
  console.log(`Better Auth API available at https://fastapi-backend-for-book.vercel.app/api/auth`);
});