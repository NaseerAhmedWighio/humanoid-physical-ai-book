# Implementation Plan: Revert Text Highlighter and Restore RAG Chatbot

**Branch**: `001-revert-text-highlighter` | **Date**: 2026-01-03 | **Spec**: [link to spec](./spec.md)
**Input**: Feature specification from `/specs/001-revert-text-highlighter/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan addresses the need to remove the recently added text highlighter functionality and restore the original RAG-based agent chatbot. The feature involves removing the TextHighlighter component that was added to the page layout and ensuring the original chatbot functionality (persisted in localStorage, positioned bottom-right of page) is fully restored. This simplifies the UI and returns to the expected RAG-based chatbot experience.

## Technical Context

**Language/Version**: JavaScript/React for frontend, Python 3.8+ for backend
**Primary Dependencies**: React, Docusaurus, FastAPI, OpenAI API for RAG functionality
**Storage**: localStorage for conversation persistence, Qdrant for vector embeddings
**Testing**: Jest for frontend, pytest for backend
**Target Platform**: Web browser (Chrome/Firefox/Safari/Edge)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <5 seconds for chatbot response, <200ms for UI interactions
**Constraints**: <200ms for UI interactions, localStorage quota limitations, offline-capable UI
**Scale/Scope**: Single-page application with content-focused RAG chatbot

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on Physical AI & Humanoid Robotics Textbook Constitution:
- ✅ Real-World Applications Focus: Features have clear real-world application in human environments (healthcare, manufacturing, etc.) - AI chatbot for robotics education
- ✅ Modular & Scalable Structure: Implementation supports 13-week course modularity - chatbot works with modular content
- ✅ Hands-On Interactive Learning: Features include Python/ROS 2 code examples and simulation tutorials - RAG chatbot supports interactive learning
- ✅ Ethical AI Integration: Ethical considerations addressed for human-robot interaction safety - AI responses follow ethical guidelines
- ✅ High-Quality Standards: Detailed explanations, visual aids, exercises, and academic references - RAG system provides quality responses
- ✅ Digital-Physical Bridge: Digital AI concepts connect to physical robot behaviors - chatbot provides information on physical robotics

**Constitution Alignment Note**: This change removes functionality that was previously included in the constitution (section 56-57: Full-Page Text Highlighting & Content Selection). This is an intentional simplification based on user feedback that the text highlighter was "not necessary" and added unwanted complexity. The core RAG-based chatbot functionality remains intact, which aligns with the Conversational AI & Contextual Understanding principle (section 41-42).

**Justification for Modification**: The user has explicitly requested removal of the text highlighter functionality, indicating it adds unnecessary complexity without significant value. This change prioritizes core functionality (RAG chatbot) over additional features, aligning with the principle of maintaining focused, high-quality educational tools.

## Project Structure

### Documentation (this feature)

```text
specs/001-revert-text-highlighter/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget.js          # RAG-based chatbot component
│   │   ├── TextHighlighter.js     # Component to be removed
│   │   └── PersonalizationToggle.js  # Kept for personalization features
│   ├── context/
│   │   └── UserPreferenceContext.js  # User preference management
│   ├── services/
│   │   ├── localStorage.js        # Storage management
│   │   └── contentSelectionService.js  # Content selection (to be removed)
│   └── theme/
│       └── Layout/
│           └── index.js           # Main layout with TextHighlighter to be removed
└── docusaurus.config.js           # Docusaurus configuration

backend/
├── src/
│   ├── api/
│   │   └── chat.py               # Chat API endpoints
│   ├── models/
│   │   └── chat_message.py       # Chat message models
│   └── services/
│       └── agent.py              # RAG agent service
└── requirements.txt              # Python dependencies
```

**Structure Decision**: Web application with frontend React components and backend API services. The TextHighlighter component will be removed from the Layout, and the original RAG-based ChatWidget will be maintained with localStorage persistence.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [None] | [N/A] | [All constitution checks passed] |
