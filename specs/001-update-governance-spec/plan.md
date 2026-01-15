# Implementation Plan: Content Personalization and Urdu Translation

**Branch**: `001-update-governance-spec` | **Date**: 2026-01-01 | **Spec**: [link]
**Input**: Feature specification from `/specs/001-update-governance-spec/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement content personalization based on user hardware preferences (mobile, laptop, physical robot) collected during signup, with a toggle to switch between personalized and original content. Additionally, implement Urdu language translation functionality with a toggle to switch between English and Urdu content. Both features will be available across the platform (excluding homepage, auth pages, and intro documentation) while preserving document structure and non-translatable elements like code blocks.

## Technical Context

**Language/Version**: JavaScript/TypeScript (frontend), Python 3.8+ (backend)
**Primary Dependencies**: Docusaurus (frontend framework), FastAPI (backend API), better-auth (authentication), PostgreSQL (user data), Qdrant (vector embeddings), React (frontend components)
**Storage**: PostgreSQL for user data and preferences, Qdrant for vector embeddings, File system (Markdown content files)
**Testing**: pytest (backend), Jest/React Testing Library (frontend)
**Target Platform**: Web-based application (browser compatible)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: Translation responses under 2 seconds, Personalization toggle responds within 1 second, Support 1000 concurrent users
**Constraints**: <200ms p95 for content delivery, Maintain accessibility for Urdu translation, Preserve document structure during translation
**Scale/Scope**: Support for 10k+ users with personalized content, Multiple hardware preference combinations, Urdu language support for all content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on Physical AI & Humanoid Robotics Textbook Constitution:
- Real-World Applications Focus: Verify all features have clear real-world application in human environments (healthcare, manufacturing, etc.) ✓ - Content personalization ensures users learn with hardware they actually have access to
- Modular & Scalable Structure: Ensure implementation supports 13-week course modularity ✓ - Personalization can be applied to any module in the course structure
- Hands-On Interactive Learning: Confirm all features include Python/ROS 2 code examples and simulation tutorials ✓ - Personalized content includes appropriate code examples for each hardware type (mobile, laptop, physical robot)
- Ethical AI Integration: Validate ethical considerations are addressed for human-robot interaction safety ✓ - Content personalization respects user preferences and data privacy
- High-Quality Standards: Verify detailed explanations (2000+ words), visual aids, exercises, and academic references ✓ - Both personalized and translated content maintains educational quality standards
- Digital-Physical Bridge: Ensure digital AI concepts connect to physical robot behaviors ✓ - Physical robot users get real-world examples while simulation users get virtual examples
- Secure User Authentication & Personalization: User accounts must support hardware preference collection during signup ✓ - Users select hardware preferences during signup for personalized content delivery
- Multilingual Content Support & Urdu Translation: Translation system must preserve document structure and support Urdu language ✓ - Translation functionality preserves document structure while translating content to Urdu

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   ├── auth.py          # Authentication endpoints
│   │   ├── translation.py   # Translation endpoints
│   │   └── user.py          # User profile and preferences
│   ├── models/
│   │   └── user.py          # User model with hardware preferences
│   └── services/
│       ├── auth_service.py
│       └── translation_service.py
└── tests/

frontend/
├── src/
│   ├── components/
│   │   ├── PersonalizationToggle/    # Personalization toggle component
│   │   ├── TranslateButton/          # Translation toggle component
│   │   ├── Auth/
│   │   │   └── Signup.js            # Signup with hardware preferences
│   │   └── UserProfile/             # User profile management
│   ├── context/
│   │   ├── PersonalizationContext.js # Personalization state management
│   │   └── BetterAuthContext.js      # Authentication context
│   ├── services/
│   │   └── translationService.js     # Translation service
│   ├── pages/
│   │   └── purpose-selection.mdx     # Hardware preference selection page
│   └── theme/
│       └── Navbar.js                # Navbar with personalization toggle
└── tests/
```

**Structure Decision**: Web application structure with frontend and backend components. The personalization and translation features are implemented as React components with corresponding backend services and API endpoints. The existing codebase already has PersonalizationToggle, translationService, and authentication components that will be enhanced to support the new requirements.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
