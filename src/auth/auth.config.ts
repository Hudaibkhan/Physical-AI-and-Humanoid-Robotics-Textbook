import { betterAuth } from "better-auth";
import { postgresAdapter } from "@better-auth/adapter-postgres";
import { Pool } from "pg";

// Initialize PostgreSQL connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  max: 20, // Maximum number of clients in the pool
  idleTimeoutMillis: 30000, // Close idle clients after 30 seconds
  connectionTimeoutMillis: 2000, // Return an error after 2 seconds if connection could not be established
});

export const auth = betterAuth({
  secret: process.env.BETTER_AUTH_SECRET_KEY || "your-secret-key-change-in-production",
  app: {
    name: "Book Platform",
  },
  database: {
    provider: "postgresql", // Switched to PostgreSQL for Neon DB
    url: process.env.DATABASE_URL || "",
    // Use PostgreSQL adapter for Neon DB
    adapter: postgresAdapter(pool, {
      user: {
        // Define the table name for users if needed
        table: "auth_user",
      },
      account: {
        // Define the table name for accounts if needed
        table: "auth_account",
      },
      session: {
        // Define the table name for sessions if needed
        table: "auth_session",
      },
    }),
  },
  socialProviders: {
    // Add social providers if needed in the future
  },
  user: {
    // Add custom fields for user metadata
    additionalFields: {
      software_background: {
        type: "string",
        required: true, // Make required as per spec
        defaultValue: "",
      },
      hardware_background: {
        type: "string",
        required: true, // Make required as per spec
        defaultValue: "",
      },
      learning_goal: {
        type: "string",
        required: true, // Make required as per spec
        defaultValue: "",
      },
    },
  },
  account: {
    // Account-related configuration
  },
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
  },
});