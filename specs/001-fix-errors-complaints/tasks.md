# Tasks: Fix Errors and Complaints

**Feature**: Fix Errors and Complaints
**Branch**: `001-fix-errors-complaints`
**Created**: 2026-01-02
**Status**: Draft

## Implementation Strategy

MVP approach: Implement User Story 1 (Personalized Content) first to establish the core personalization functionality, then build upon it with the other stories. Each user story should be independently testable and deliver value.

## Dependencies

- User Story 1 (Personalization) must be completed before User Story 3 (TextHighlighter) can be fully tested
- User Story 2 (Chatbot Persistence) is independent but may share localStorage utilities
- User Story 4 (Search) is independent but requires backend API fixes

## Parallel Execution Examples

- [P] Tasks T005-T008 (User Preferences model and services) can run in parallel with T010-T013 (Chat Conversation model and services)
- [P] Frontend components for personalization can be developed in parallel with backend API endpoints
- [P] Search API fixes can be done in parallel with other stories

## Phase 1: Setup

- [ ] T001 Set up development environment per quickstart.md
- [ ] T002 Install required dependencies (React, Docusaurus, FastAPI, Qdrant)
- [ ] T003 Configure project structure with frontend and backend directories
- [ ] T004 Set up testing frameworks (Jest for frontend, pytest for backend)

## Phase 2: Foundational

- [X] T005 Create User Preferences model in backend/src/models/user_preferences.py
- [X] T006 Create Chat Conversation model in backend/src/models/chat_conversation.py
- [X] T007 Create Personalized Content model in backend/src/models/personalized_content.py
- [X] T008 Create Search Results model in backend/src/models/search_results.py
- [X] T009 Create User Preferences service in backend/src/services/user_preferences_service.py
- [X] T010 Create Chat Conversation service in backend/src/services/chat_conversation_service.py
- [X] T011 Create Personalized Content service in backend/src/services/personalized_content_service.py
- [X] T012 Create Search service in backend/src/services/search_service.py
- [X] T013 Create localStorage utility functions in frontend/src/services/localStorage.js
- [X] T014 Create user preference context in frontend/src/context/UserPreferenceContext.js

## Phase 3: [US1] Fix Personalized Content Functionality

**Goal**: Enable personalized content that converts MD files to HTML based on user hardware preferences, with toggle functionality to reset to original content

**Independent Test Criteria**: Can enable/disable the personalization toggle and verify content changes according to user preferences and resets when disabled

**Tasks**:

- [X] T015 [US1] Create PersonalizationToggle component in frontend/src/components/PersonalizationToggle.js
- [X] T016 [US1] Create PersonalizedContent component in frontend/src/components/PersonalizedContent.js
- [X] T017 [US1] Implement GET /api/personalization/content/{contentId} endpoint in backend/src/api/personalization.py
- [X] T018 [US1] Implement POST /api/personalization/toggle endpoint in backend/src/api/personalization.py
- [X] T019 [US1] Create content personalization logic in backend/src/services/personalized_content_service.py
- [X] T020 [US1] Implement hardware preference mapping logic (mobile/laptop/physical robot) in frontend/src/services/personalizationService.js
- [X] T021 [US1] Add personalization toggle to navigation sidebar in frontend/src/components/NavigationSidebar.js
- [X] T022 [US1] Create MD to HTML personalization transformer in frontend/src/services/mdToHtmlTransformer.js
- [ ] T023 [US1] Test personalization toggle functionality with acceptance scenarios
- [ ] T024 [US1] Verify content resets to original when personalization is disabled

## Phase 4: [US2] Fix Chatbot Conversation Persistence

**Goal**: Ensure chatbot conversations persist across page refreshes using localStorage or session storage

**Independent Test Criteria**: Can start a conversation, refresh the page, and verify conversation history is preserved

**Tasks**:

- [X] T025 [US2] Update ChatWidget component to use localStorage for conversation persistence in frontend/src/components/ChatWidget.js
- [X] T026 [US2] Create conversation persistence service in frontend/src/services/conversationPersistence.js
- [X] T027 [US2] Implement localStorage quota management for conversations in frontend/src/services/conversationPersistence.js
- [X] T028 [US2] Update POST /api/chat/conversation endpoint to support client-side persistence in backend/src/api/chat.py
- [X] T029 [US2] Update GET /api/chat/conversation/{conversationId} endpoint to work with persisted conversations in backend/src/api/chat.py
- [X] T030 [US2] Create conversation cleanup utility to handle localStorage limits in frontend/src/services/conversationPersistence.js
- [ ] T031 [US2] Test conversation persistence across page refreshes
- [ ] T032 [US2] Test conversation accessibility across multiple browser tabs

## Phase 5: [US3] Fix TextHighlighter to Work on Full Page Content

**Goal**: Enable TextHighlighter to work on entire page content, not just first 10-15 lines

**Independent Test Criteria**: Can highlight text from different parts of the page (beginning, middle, end) and verify selected content is properly sent to chatbot

**Tasks**:

- [X] T033 [US3] Update TextHighlighter component to select from entire page in frontend/src/components/TextHighlighter.js
- [X] T034 [US3] Create full-page content selection utility in frontend/src/services/contentSelectionService.js
- [X] T035 [US3] Update chat integration to handle full-page selections in frontend/src/components/ChatWidget.js
- [ ] T036 [US3] Test text selection from middle of long pages
- [ ] T037 [US3] Test text selection from end of long pages
- [ ] T038 [US3] Verify selected content is properly sent to chatbot for analysis
- [ ] T039 [US3] Handle edge case of very long page content selection

## Phase 6: [US4] Fix CustomSearch.js Functionality

**Goal**: Make CustomSearch.js work like SearchModal.js, returning proper search results without 404 errors

**Independent Test Criteria**: Can perform searches through CustomSearch.js and verify results are returned without 404 errors

**Tasks**:

- [X] T040 [US4] Compare CustomSearch.js with SearchModal.js to identify differences in frontend/src/components/CustomSearch.js
- [X] T041 [US4] Fix backend API integration in CustomSearch.js to match SearchModal.js in frontend/src/components/CustomSearch.js
- [X] T042 [US4] Implement GET /api/search endpoint in backend/src/api/search.py
- [X] T043 [US4] Test CustomSearch.js with various queries to ensure no 404 errors
- [X] T044 [US4] Verify search results consistency between CustomSearch.js and SearchModal.js
- [X] T045 [US4] Add error handling for backend search API unavailability in frontend/src/components/CustomSearch.js

## Phase 7: Polish & Cross-Cutting Concerns

- [X] T046 Add localStorage quota error handling across all features
- [ ] T047 Create fallback mechanisms for when personalization or search services are unavailable
- [ ] T048 Add proper error boundaries and user feedback for all components
- [ ] T049 Update documentation to reflect the implemented fixes
- [ ] T050 Perform integration testing of all features together
- [ ] T051 Run performance tests to ensure <200ms for personalization toggle, <500ms for search, <1000ms for chat responses
- [ ] T052 Verify all acceptance scenarios from user stories are satisfied
- [ ] T053 Update constitution compliance checks to reflect implemented changes