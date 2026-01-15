# Quickstart Guide: Auth and AI Functionality Updates

**Feature**: 002-auth-ai-functionality
**Date**: 2025-12-25

## Overview

This guide provides instructions for setting up and implementing the authentication and AI functionality updates for the humanoid robotics textbook website.

## Prerequisites

- Python 3.8+
- Node.js 16+ (for frontend development)
- PostgreSQL 12+
- Qdrant vector database
- Access to OpenAI API (or compatible service)

## Environment Setup

### Backend Setup

1. **Install Python dependencies:**
```bash
cd backend
pip install -r requirements.txt
```

2. **Set up environment variables:**
```bash
# backend/.env
DATABASE_URL=postgresql://user:password@localhost:5432/humanoid_textbook
QDRANT_URL=http://localhost:6333
OPENAI_API_KEY=your_openai_api_key
JWT_SECRET_KEY=your_jwt_secret_key
```

3. **Run database migrations:**
```bash
cd backend
python -m src.database --create-tables
```

4. **Start the backend server:**
```bash
cd backend
uvicorn src.main:app --reload --port 8000
```

### Frontend Setup

1. **Install Node dependencies:**
```bash
cd frontend
npm install
```

2. **Set up environment variables:**
```bash
# frontend/.env
REACT_APP_API_BASE_URL=http://localhost:8000
```

3. **Start the frontend development server:**
```bash
cd frontend
npm start
```

## Implementation Steps

### 1. Authentication Error Handling

**Files to update:**
- `backend/src/services/auth_service.py`
- `backend/src/api/better_auth.py`
- `frontend/src/context/BetterAuthContext.js`

**Key changes:**
- Add defensive coding to prevent "undefined property" errors
- Implement proper response validation
- Add fallback values for missing properties

### 2. Database Connection Management

**Files to update:**
- `backend/src/database.py`
- `backend/src/services/auth_service.py`

**Key changes:**
- Implement connection pooling
- Add retry mechanisms for transient failures
- Add proper error handling for database operations

### 3. AI Conversation Context

**Files to update:**
- `backend/src/services/agent.py`
- `backend/src/api/chat.py`
- `frontend/src/components/ChatWidget/index.js`

**Key changes:**
- Implement session-based conversation context
- Add source attribution to AI responses
- Maintain context across multiple exchanges

### 4. Text Selection Feature

**Files to update:**
- `frontend/src/components/TextHighlighter/index.js`
- `frontend/src/theme/Layout/index.js`

**Key changes:**
- Implement text selection detection
- Add "Ask Question" button positioning
- Integrate with chat API

### 5. Auth Page Restrictions

**Files to update:**
- `frontend/src/theme/Layout/index.js`
- `frontend/src/components/TextHighlighter/index.js`

**Key changes:**
- Conditionally render interactive features on auth pages
- Ensure "Ask Question" button doesn't appear on signin/signup

## API Endpoints

### Authentication
- `POST /v1/auth/register` - Register new user
- `POST /v1/auth/login` - Authenticate user
- `GET /v1/auth/me` - Get current user
- `POST /v1/auth/update-preferences` - Update user preferences
- `POST /v1/better-auth/register` - Better Auth registration
- `POST /v1/better-auth/login` - Better Auth login

### Chat/AI
- `POST /v1/chat/sessions` - Create chat session
- `POST /v1/chat/sessions/{session_id}/messages` - Send message
- `POST /v1/chat/ask-from-selection` - Ask about selected text
- `GET /v1/chat/sessions` - Get user sessions
- `GET /v1/chat/sessions/{session_id}/messages` - Get session messages

## Testing

### Backend Tests
```bash
cd backend
pytest tests/
```

### Frontend Tests
```bash
cd frontend
npm test
```

## Configuration

### Database Configuration
The application uses PostgreSQL for user data and Qdrant for vector embeddings. Ensure both services are running and properly configured in your environment variables.

### AI Service Configuration
Configure your OpenAI API key or compatible service. The AI service handles both contextual question-answering and source attribution.

## Deployment

### Production Build
```bash
# Backend
cd backend
gunicorn src.main:app -w 4 -b 0.0.0.0:8000

# Frontend
cd frontend
npm run build
```

### Environment Variables for Production
Ensure all required environment variables are set in your production environment, including database URLs, API keys, and security settings.

## Troubleshooting

### Common Issues

1. **"Cannot read properties of undefined" errors:**
   - Check authentication response handling in frontend
   - Ensure all API responses include proper structure validation

2. **Database connection failures:**
   - Verify PostgreSQL is running and accessible
   - Check connection string format in environment variables

3. **AI responses taking too long:**
   - Verify OpenAI API key is valid
   - Check network connectivity to AI service
   - Review rate limiting settings

### Debugging Tips

- Enable debug logging in backend by setting `LOG_LEVEL=DEBUG`
- Use browser developer tools to inspect API requests and responses
- Check server logs for error details and stack traces