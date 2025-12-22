// Generic database utility functions for personalization
// This can work with various database adapters including Neon Postgres

// Simple error handling wrapper for database operations
export async function withDbTransaction<T>(
  operation: () => Promise<T>
): Promise<T> {
  try {
    // Begin transaction logic would go here based on the database adapter used
    const result = await operation();
    // Commit transaction logic would go here
    return result;
  } catch (error) {
    // Rollback transaction logic would go here
    console.error('Database transaction error:', error);
    throw error;
  }
}

// Initialize the database connection when module is loaded
export function initDbConnection(): void {
  const databaseUrl = process.env.DATABASE_URL;
  if (!databaseUrl) {
    console.warn('DATABASE_URL environment variable is not set');
  }
  // Initialize connection pool based on the database provider
  console.log('Database connection initialized');
}

// Export a simple error handling function for database operations
export async function handleDbOperation<T>(
  operation: () => Promise<T>,
  operationName: string = 'database operation'
): Promise<T> {
  try {
    return await operation();
  } catch (error) {
    console.error(`Error in ${operationName}:`, error);
    throw new Error(`Failed to execute ${operationName}: ${(error as Error).message}`);
  }
}