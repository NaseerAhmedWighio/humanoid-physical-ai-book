# Implementation Tasks: Revert Text Highlighter and Restore RAG Chatbot

**Feature**: Revert Text Highlighter and Restore RAG Chatbot
**Feature Spec**: [spec.md](./spec.md)
**Plan**: [plan.md](./plan.md)
**Generated**: 2026-01-03

## Implementation Strategy

This feature will remove the text highlighter functionality and restore the original RAG-based agent chatbot. The implementation follows a phased approach with User Story 1 (P1) as the MVP, followed by User Story 2 (P1), and finally User Story 3 (P2).

## Dependencies

- User Story 1 must complete before User Story 2 (removing TextHighlighter component)
- User Story 1 completion is required for User Story 3 (chat persistence)
- All user stories depend on foundational setup tasks

## Parallel Execution Opportunities

- [US1] Tasks T005-T008 (different files): ChatWidget.js, localStorage.js, contentSelectionService.js, Layout/index.js
- [US2] Tasks T012-T013 (component removal): TextHighlighter.js deletion, Layout cleanup

---

## Phase 1: Setup Tasks

Setup tasks for project initialization and environment configuration.

### Goal
Initialize the project environment and prepare for implementation.

### Independent Test Criteria
Project can be built and run without errors after setup tasks are completed.

### Tasks

- [x] T001 Create backup of current files before making changes
- [x] T002 Verify current functionality works as expected before changes

---

## Phase 2: Foundational Tasks

Foundational tasks that block all user stories.

### Goal
Prepare the codebase for the revert implementation by identifying dependencies and ensuring core functionality remains intact.

### Independent Test Criteria
Core chatbot functionality remains accessible and functional.

### Tasks

- [x] T003 Identify all dependencies of TextHighlighter component to ensure no other components use it
- [x] T004 Verify ChatWidget.js functionality is independent of TextHighlighter component

---

## Phase 3: User Story 1 - Restore Original RAG Chatbot (P1)

Users should have access to the original RAG-based agent chatbot functionality without the additional text highlighter interface that was recently added. The chatbot should function as the primary AI assistant for answering questions about the humanoid robotics content using retrieval augmented generation.

### Goal
Ensure the original RAG-based chatbot functionality is available and working properly.

### Independent Test Criteria
Can be fully tested by interacting with the chatbot interface and verifying it responds to questions about the content using RAG capabilities, delivering the expected AI-powered assistance without the text highlighter.

### Tasks

- [x] T005 [P] [US1] Verify ChatWidget.js still functions as expected without TextHighlighter
- [x] T006 [P] [US1] Ensure ChatWidget maintains its position bottom-right of page
- [x] T007 [P] [US1] Verify ChatWidget still connects to backend for RAG responses
- [x] T008 [P] [US1] Confirm ChatWidget uses localStorage for conversation persistence

---

## Phase 4: User Story 2 - Remove Text Highlighter Component (P1)

The text highlighter component that was added to the bottom of pages should be removed completely, simplifying the UI and removing the unnecessary "Ask Question" button that appeared when text was selected.

### Goal
Completely remove the text highlighter component and its integration from the application.

### Independent Test Criteria
Can be tested by selecting text on any page and verifying that the "Ask Question" button does not appear, confirming the text highlighter functionality has been removed.

### Tasks

- [x] T009 [US2] Remove TextHighlighter import from Layout/index.js
- [x] T010 [US2] Remove TextHighlighter JSX component usage from Layout/index.js
- [x] T011 [US2] Remove handleSelectionAsk function from Layout/index.js if no longer needed
- [x] T012 [P] [US2] Delete frontend/src/components/TextHighlighter.js file
- [x] T013 [P] [US2] Delete frontend/src/services/contentSelectionService.js file if only used by TextHighlighter (SKIPPED - kept as ChatWidget.js also uses getSelectedText function for "Send Selected Text" feature)
- [x] T014 [US2] Remove any CSS styling related to text highlighter functionality
- [x] T015 [US2] Test that no "Ask Question" button appears when text is selected (verified - TextHighlighter component removed from Layout)

---

## Phase 5: User Story 3 - Maintain Chatbot Persistence (P2)

The chatbot conversation history should continue to persist across page refreshes using localStorage, maintaining the user's conversation context.

### Goal
Ensure chat conversation history persists across page refreshes using localStorage as it did before.

### Independent Test Criteria
Can be tested by starting a conversation, refreshing the page, and verifying the conversation history is preserved.

### Tasks

- [x] T016 [US3] Verify localStorage service properly saves conversation history in ChatWidget.js
- [x] T017 [US3] Test conversation history preservation across page refreshes
- [x] T018 [US3] Confirm localStorage key 'chat_conversation' is used correctly
- [x] T019 [US3] Validate that conversation persistence works after TextHighlighter removal

---

## Phase 6: Polish & Cross-Cutting Concerns

Final tasks to ensure quality, testing, and documentation.

### Goal
Complete the implementation with proper testing, cleanup, and validation.

### Independent Test Criteria
All functionality works as expected and no artifacts of the removed text highlighter remain.

### Tasks

- [x] T020 [P] Run frontend build to ensure no compilation errors after changes
- [x] T021 [P] Test that chatbot functionality works on different pages
- [x] T022 [P] Verify no console errors appear after TextHighlighter removal
- [x] T023 [P] Confirm chatbot still uses RAG capabilities for responses
- [x] T024 [P] Test end-to-end workflow: page navigation, chat interaction, persistence
- [x] T025 [P] Clean up any unused imports or references to TextHighlighter
- [x] T026 [P] Update any documentation that referenced the removed text highlighter
- [x] T027 [P] Perform final verification that all acceptance criteria from spec are met