# Quickstart: Revert Text Highlighter and Restore RAG Chatbot

## Overview
This guide provides instructions for implementing the revert of text highlighter functionality and restoring the original RAG-based agent chatbot.

## Prerequisites
- Node.js 18+ installed
- Python 3.8+ installed
- npm or yarn package manager
- Git for version control

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd humanoid-ai-book
```

### 2. Install Dependencies
```bash
# Frontend dependencies
cd frontend
npm install

# Backend dependencies
cd ../backend
pip install -r requirements.txt
```

### 3. Environment Configuration
```bash
# Copy environment template
cp .env.example .env

# Update environment variables as needed
# OPENAI_API_KEY, QDRANT_URL, etc.
```

## Implementation Steps

### Step 1: Remove Text Highlighter Component

1. **Delete the TextHighlighter component:**
```bash
rm frontend/src/components/TextHighlighter.js
```

2. **Remove TextHighlighter from Layout:**
Edit `frontend/src/theme/Layout/index.js` and remove:
- Import statement: `import TextHighlighter from '@site/src/components/TextHighlighter';`
- The component usage in JSX: `<TextHighlighter onSelectionAsk={handleSelectionAsk} />`

3. **Remove related service if unused elsewhere:**
```bash
rm frontend/src/services/contentSelectionService.js
```

### Step 2: Verify ChatWidget Functionality

1. **Ensure ChatWidget is properly integrated:**
- Located at `frontend/src/components/ChatWidget.js`
- Should use localStorage for conversation persistence
- Should connect to backend chat API

2. **Check Layout integration:**
The ChatWidget should remain in the Layout component with proper positioning (bottom-right).

### Step 3: Test the Implementation

1. **Start the development server:**
```bash
# Backend
cd backend
uvicorn main:app --reload

# Frontend
cd frontend
npm run start
```

2. **Verify functionality:**
- Text highlighter no longer appears when selecting text
- Chatbot appears bottom-right of page
- Chatbot maintains conversation history across page refreshes
- RAG functionality works as expected

## API Endpoints (Backend)

### Chat Endpoints
- `POST /api/chat/conversation` - Send message and get RAG response
- `GET /api/chat/conversation/{id}` - Get conversation history

### Content Endpoints
- `POST /api/content/search` - Search content for RAG context
- `GET /api/content/{id}` - Get specific content for context

## Environment Variables

### Required Variables
```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_instance_url
QDRANT_API_KEY=your_qdrant_api_key
```

### Optional Variables
```
CHAT_MODEL=gpt-4  # Default: gpt-3.5-turbo
EMBEDDING_MODEL=text-embedding-ada-002  # Default: text-embedding-ada-002
```

## Data Storage

### LocalStorage Keys
- `chat_conversation` - Stores current conversation
- `userPreferences` - Stores user preferences
- `userPreferences.hardwarePreference` - User's hardware preference
- `userPreferences.personalizationEnabled` - Personalization toggle state

### Size Limits
- LocalStorage quota: 5MB (browser-dependent)
- Individual conversation size: <1MB recommended
- Message history: Limited to prevent quota issues

## Troubleshooting

### Common Issues

**Issue**: Chatbot not responding
**Solution**: Verify API keys are set correctly and backend is running

**Issue**: Conversation history not persisting
**Solution**: Check localStorage permissions and quota limits

**Issue**: Text highlighter still appears
**Solution**: Verify component removal from Layout and rebuild frontend

### Debugging Steps

1. **Check browser console for errors**
2. **Verify network requests to backend**
3. **Confirm localStorage contains expected data**
4. **Review backend logs for API issues**

## Performance Considerations

### Response Times
- Chat response: <5 seconds
- UI interactions: <200ms
- LocalStorage operations: <50ms

### Optimization Tips
- Limit conversation history size to prevent localStorage bloat
- Implement proper cleanup of old conversations
- Use efficient data structures for message storage

## Testing Checklist

- [ ] Text highlighter does not appear when text is selected
- [ ] Chatbot is visible in bottom-right corner
- [ ] Chatbot responses are generated via RAG
- [ ] Conversation history persists across page refreshes
- [ ] User preferences are maintained
- [ ] No console errors appear
- [ ] API requests are successful
- [ ] localStorage quota is not exceeded

## Rollback Plan

If issues occur after implementation:

1. **Restore from backup branch if available**
2. **Revert specific file changes**
3. **Verify all functionality before redeployment**