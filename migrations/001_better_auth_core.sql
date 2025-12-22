-- Better Auth Core Schema
-- This migration creates the 4 core tables required by Better Auth
-- Reference: https://www.better-auth.com/docs/adapters/postgresql

-- User table (managed by Better Auth)
CREATE TABLE IF NOT EXISTS "user" (
  "id" VARCHAR(255) PRIMARY KEY,
  "name" VARCHAR(255) NOT NULL,
  "email" VARCHAR(255) NOT NULL UNIQUE,
  "emailVerified" BOOLEAN NOT NULL DEFAULT FALSE,
  "image" TEXT,
  "createdAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Session table (managed by Better Auth)
CREATE TABLE IF NOT EXISTS "session" (
  "id" VARCHAR(255) PRIMARY KEY,
  "userId" VARCHAR(255) NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
  "token" VARCHAR(255) NOT NULL UNIQUE,
  "expiresAt" TIMESTAMP NOT NULL,
  "ipAddress" VARCHAR(45),
  "userAgent" TEXT,
  "createdAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS "idx_session_userId" ON "session"("userId");
CREATE INDEX IF NOT EXISTS "idx_session_token" ON "session"("token");

-- Account table (managed by Better Auth - for OAuth and password storage)
CREATE TABLE IF NOT EXISTS "account" (
  "id" VARCHAR(255) PRIMARY KEY,
  "userId" VARCHAR(255) NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
  "accountId" VARCHAR(255) NOT NULL,
  "providerId" VARCHAR(255) NOT NULL,
  "accessToken" TEXT,
  "refreshToken" TEXT,
  "accessTokenExpiresAt" TIMESTAMP,
  "refreshTokenExpiresAt" TIMESTAMP,
  "scope" TEXT,
  "idToken" TEXT,
  "password" TEXT,
  "createdAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS "idx_account_userId" ON "account"("userId");
CREATE UNIQUE INDEX IF NOT EXISTS "idx_account_provider" ON "account"("providerId", "accountId");

-- Verification table (managed by Better Auth)
CREATE TABLE IF NOT EXISTS "verification" (
  "id" VARCHAR(255) PRIMARY KEY,
  "identifier" VARCHAR(255) NOT NULL,
  "value" VARCHAR(255) NOT NULL,
  "expiresAt" TIMESTAMP NOT NULL,
  "createdAt" TIMESTAMP,
  "updatedAt" TIMESTAMP
);

CREATE INDEX IF NOT EXISTS "idx_verification_identifier" ON "verification"("identifier");
