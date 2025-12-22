# Auth API Contracts

## User Registration
- **Endpoint**: `POST /api/auth/register`
- **Request**:
  - email: string (required)
  - password: string (required, min 8 chars)
  - software_background: string (required)
  - hardware_background: string (required)
  - learning_goal: string (required)
- **Response**:
  - success: boolean
  - user: { id, email, software_background, hardware_background, learning_goal }
  - session_token: string (optional, if auto-login enabled)

## User Login
- **Endpoint**: `POST /api/auth/login`
- **Request**:
  - email: string (required)
  - password: string (required)
- **Response**:
  - success: boolean
  - user: { id, email, software_background, hardware_background, learning_goal }
  - session_token: string

## User Profile
- **Endpoint**: `GET /api/auth/profile`
- **Auth Required**: Bearer token
- **Response**:
  - user: { id, email, software_background, hardware_background, learning_goal }

## Update Profile
- **Endpoint**: `PUT /api/auth/profile`
- **Auth Required**: Bearer token
- **Request**:
  - software_background: string (optional)
  - hardware_background: string (optional)
  - learning_goal: string (optional)
- **Response**:
  - success: boolean
  - user: { id, email, software_background, hardware_background, learning_goal }