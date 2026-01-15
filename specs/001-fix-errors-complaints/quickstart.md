# Quickstart Guide for Fix Errors and Complaints Feature

## Setup

1. **Prerequisites**:
   - Node.js 18+ for frontend
   - Python 3.8+ for backend
   - Access to OpenAI API for RAG functionality
   - Qdrant vector database running

2. **Installation**:
   ```bash
   # Frontend setup
   cd frontend
   npm install

   # Backend setup
   cd backend
   pip install -r requirements.txt
   ```

3. **Environment variables**:
   ```bash
   # Create .env files in both frontend and backend
   # with necessary API keys and database connections
   ```

## Running the Application

1. **Start backend**:
   ```bash
   cd backend
   python -m uvicorn main:app --reload
   ```

2. **Start frontend**:
   ```bash
   cd frontend
   npm run start
   ```

## Key Features

1. **Personalized Content**:
   - Enable personalization toggle in user settings
   - Content will adapt based on selected hardware preference
   - Disable toggle to return to original content

2. **Chatbot Persistence**:
   - Conversations automatically saved to localStorage
   - Refresh page - conversation history preserved
   - Switch between tabs - conversation accessible

3. **Full-Page Text Selection**:
   - Select text from anywhere on the page
   - Use "Ask Question" button to send to chatbot
   - Works with all page content, not just first lines

4. **Unified Search**:
   - Search functionality consistent across all components
   - No more 404 errors on valid searches
   - Same results whether using CustomSearch or SearchModal

## Development

- Frontend components in `frontend/src/components/`
- Backend APIs in `backend/src/api/`
- React components for personalization in `PersonalizationToggle`, `PersonalizedContent`
- Chat functionality in `ChatWidget` component
- Search components in `CustomSearch`, `SearchModal`