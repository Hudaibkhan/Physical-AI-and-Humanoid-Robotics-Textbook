import './load-env.js';
import { auth } from './lib/auth.js';

console.log('🧪 Testing Better Auth configuration...');
console.log('Database URL:', process.env.DATABASE_URL ? '✓ Set' : '✗ Missing');
console.log('Better Auth Secret:', process.env.BETTER_AUTH_SECRET ? '✓ Set' : '✗ Missing');
console.log('Better Auth URL:', process.env.BETTER_AUTH_URL);

// Test creating a simple request
try {
  const testRequest = new Request('http://localhost:8000/api/auth/get-session', {
    method: 'GET',
    headers: new Headers({
      'Content-Type': 'application/json'
    })
  });

  console.log('\n✓ Request object created successfully');
  console.log('Request URL:', testRequest.url);
  console.log('Request method:', testRequest.method);

  // Try to get the handler
  const { toNodeHandler } = await import('better-auth/node');
  const handler = toNodeHandler(auth);

  console.log('\n✓ Better Auth handler created successfully');

  // Try calling the handler
  console.log('\n🔄 Calling handler with test request...');
  const response = await handler(testRequest);

  console.log('\n✓ Handler response received');
  console.log('Response status:', response.status);
  const text = await response.text();
  console.log('Response body:', text);

} catch (error) {
  console.error('\n✗ Error occurred:');
  console.error('Message:', error.message);
  console.error('Stack:', error.stack);
}

process.exit(0);
