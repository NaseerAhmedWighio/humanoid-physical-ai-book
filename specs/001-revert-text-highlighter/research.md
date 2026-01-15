# Research: Revert Text Highlighter and Restore RAG Chatbot

## Overview
This research document addresses the requirements to remove the text highlighter functionality and restore the original RAG-based agent chatbot as specified in the feature specification.

## Current State Analysis

### Text Highlighter Component
- Located in `frontend/src/components/TextHighlighter.js`
- Added to the Layout in `frontend/src/theme/Layout/index.js`
- Provides "Ask Question" button when text is selected
- Was added as an enhancement but creates UI complexity

### RAG Chatbot Component
- Located in `frontend/src/components/ChatWidget.js`
- Positioned bottom-right of page
- Uses localStorage for conversation persistence
- Connects to backend for RAG (Retrieval Augmented Generation) responses
- Should maintain its core functionality during the revert process

## Implementation Decisions

### Decision: Remove TextHighlighter Component
- **Rationale**: The user explicitly requested removal of the text highlighter as it was not necessary and added unwanted complexity
- **Approach**: Completely remove the TextHighlighter component from the Layout and delete the component file
- **Impact**: Simplifies UI and removes the "Ask Question" button that appears when text is selected

### Decision: Maintain ChatWidget Functionality
- **Rationale**: The original RAG-based chatbot is the core functionality that users expect
- **Approach**: Keep the ChatWidget component intact with localStorage persistence
- **Impact**: Preserves conversation history and RAG capabilities

### Decision: Update Layout Integration
- **Rationale**: The Layout currently includes the TextHighlighter component which needs to be removed
- **Approach**: Remove the TextHighlighter import and JSX from the Layout component
- **Impact**: Clean layout without text selection functionality

## Technical Considerations

### Files to Modify
1. `frontend/src/theme/Layout/index.js` - Remove TextHighlighter integration
2. `frontend/src/components/TextHighlighter.js` - Delete component file
3. `frontend/src/services/contentSelectionService.js` - Delete service if only used by TextHighlighter
4. Potentially other related CSS files for text highlighter styling

### Files to Preserve
1. `frontend/src/components/ChatWidget.js` - Core RAG chatbot functionality
2. `frontend/src/services/localStorage.js` - Conversation persistence
3. Backend API endpoints for chat functionality
4. User preference contexts and related services

## Dependencies and Integration Points

### Frontend Dependencies
- React components and hooks
- Docusaurus layout system
- LocalStorage service for persistence
- CSS styling for components

### Backend Dependencies
- Chat API endpoints for RAG functionality
- Qdrant vector database for content retrieval
- OpenAI API for response generation

## Risk Assessment

### Low Risk Items
- Removing unused component - straightforward deletion
- Maintaining existing chat functionality - no changes required

### Medium Risk Items
- Ensuring no other components depend on TextHighlighter - requires verification
- Preserving chatbot functionality during layout changes - requires testing

## Alternatives Considered

### Alternative 1: Keep Text Highlighter but Disable
- **Rejected**: Goes against user's explicit request to remove the functionality completely
- **Reason**: User clearly stated it was "not necessary" and wanted to "remove it"

### Alternative 2: Modify Text Highlighter Instead of Removing
- **Rejected**: Doesn't address the core request to remove the component
- **Reason**: User wants to revert to the original state before the text highlighter was added

### Alternative 3: Replace with Different UI Pattern
- **Rejected**: Adds complexity instead of simplifying as requested
- **Reason**: User wants the original RAG chatbot experience, not a different enhancement

## Testing Strategy

### Pre-Implementation Verification
- Confirm TextHighlighter is the only component using contentSelectionService
- Verify ChatWidget functionality is independent of TextHighlighter
- Document current behavior for regression testing

### Post-Implementation Verification
- Confirm TextHighlighter no longer appears when text is selected
- Verify ChatWidget still functions with localStorage persistence
- Test conversation history preservation across page refreshes
- Ensure no console errors or broken functionality

## Rollback Plan

If issues arise after implementation:
- Keep the original files in a backup branch
- Revert changes if the RAG chatbot functionality is impacted
- Maintain a clean separation between the reverted functionality and the original system