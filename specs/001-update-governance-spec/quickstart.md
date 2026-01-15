# Quickstart Guide: Content Personalization and Urdu Translation

## Overview
This guide covers the implementation of content personalization based on user hardware preferences and Urdu language translation functionality.

## Prerequisites
- Node.js 18+ and npm for frontend development
- Python 3.8+ for backend development
- PostgreSQL database for user data
- Qdrant vector database for embeddings
- Better-auth configured for authentication

## Setting Up Personalization

### 1. User Signup with Hardware Preferences
The signup form already includes hardware preference selection:
- Mobile device access
- Laptop/computer access
- Physical robot access
- Other hardware description
- Web development experience level

### 2. Personalization Context
The PersonalizationContext manages user preferences and provides:
- `isPersonalized` state for toggle status
- `togglePersonalization()` function
- `getPersonalizedContent()` function
- `activeFilters` for content filtering

### 3. Personalization Toggle Component
The PersonalizationToggle component:
- Allows users to switch between personalized and original content
- Shows user's selected hardware preferences
- Is available on all pages except homepage, auth pages, and /docs/intro

## Setting Up Urdu Translation

### 1. Translation Service
The translationService provides:
- English-to-Urdu translation
- Urdu-to-English translation
- Caching mechanism with fallback dictionary
- Dynamic translation of content areas

### 2. Translation Toggle
The translation functionality:
- Preserves document structure and heading tags
- Translates main content areas and sidebars
- Preserves code blocks and technical diagrams
- Works dynamically without page reloads

## API Endpoints

### Backend Endpoints
- `POST /v1/translation/translate` - Translation API
- `GET /api/user/preferences` - Get user preferences
- `PUT /api/user/preferences` - Update user preferences

### Frontend Integration
- Translation service integrated via context providers
- Personalization context integrated with authentication context
- Dynamic content updates without page refresh

## Running the Application

### Backend
```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --reload
```

### Frontend
```bash
cd frontend
npm install
npm run start
```

## Testing the Features

1. Register a new user and select hardware preferences
2. Navigate to content pages to see personalized content
3. Use the PersonalizationToggle to switch between personalized/original content
4. Use the translation toggle to switch between English/Urdu content
5. Verify that document structure is preserved during translation
6. Confirm that code blocks remain unchanged during translation