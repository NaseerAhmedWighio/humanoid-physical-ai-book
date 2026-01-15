<!-- SYNC IMPACT REPORT
Version change: 1.3.0 → 1.4.0
Added sections: Content Personalization from Markdown Files, Chatbot Conversation Persistence, Full-Page Text Highlighting, Unified Search Functionality
Removed sections: None
Modified sections: None
Templates requiring updates:
- ✅ .specify/templates/plan-template.md (Constitution Check section)
- ✅ .specify/templates/spec-template.md (scope/requirements alignment)
- ✅ README.md or other runtime guidance docs
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### Real-World Applications Focus
All content must focus on real-world applications where humanoid robots interact in human environments (e.g., healthcare, manufacturing, service industries). Theoretical concepts must be grounded in practical implementation scenarios that demonstrate tangible value in human-robot interaction contexts.

### Modular & Scalable Structure
All content must be modular and scalable for a 13-week course structure. Each module must be independently consumable while building cohesively toward comprehensive understanding. Content must support different learning paces and allow for customization based on institutional needs.

### Hands-On Interactive Learning
All theoretical content must be paired with hands-on interactivity through Python code examples, ROS 2 snippets, simulation tutorials in Gazebo/Unity, and NVIDIA Isaac workflows. Every concept must have practical implementation examples that readers can execute and modify.

### Ethical AI Integration
Ethical AI discussions must be integrated throughout all content, prioritizing safety in human-robot interactions and bias in perception systems. Every technical chapter must include ethical considerations and responsible AI practices.

### High-Quality Standards
All content must meet high-quality standards with detailed explanations (at least 2000 words per major section), visual aids (Mermaid diagrams for architectures, PlantUML for robot models), quizzes/exercises at the end of each subsection, real-world case studies (e.g., Boston Dynamics Atlas robot applications), and references to academic papers/IEEE standards.

### Digital-Physical Bridge
Content must emphasize the bridge between digital AI agents and physical embodied intelligence, demonstrating how abstract AI concepts translate to physical robot behaviors and real-world interactions.

### Secure User Authentication & Personalization
All user-facing systems must implement secure authentication using better-auth with MCP server integration. User accounts must support hardware preference collection during signup (mobile, laptop, physical robot) to enable personalized content delivery. User data must be stored securely in compliance with privacy regulations and best practices. Authentication flows must include proper error handling with user-friendly messages, and all API responses must be defensively coded to prevent undefined property access errors.

### Robust Authentication & Data Management
All authentication and user data operations must implement comprehensive error handling and validation. User registration and login flows must gracefully handle edge cases, network failures, and data validation errors. Database connections must be properly configured with appropriate retry mechanisms, and user data must be reliably stored and retrieved with integrity checks. Account creation processes must provide clear feedback to users and handle all response structures appropriately to prevent undefined property errors.

### Conversational AI & Contextual Understanding
All AI-powered features must implement proper conversation context management and state preservation. The "ask question" functionality for selected text must integrate with a conversational agent that maintains context across multiple exchanges. AI responses must be generated with proper source attribution and should reference previous conversation history to provide coherent, contextual answers. The system must ensure that interactive features are appropriately disabled on authentication pages to maintain user experience consistency.

### User Preference-Based Content Personalization
All content delivery must support personalization based on user hardware preferences collected during signup. The system must offer three hardware options: mobile device, laptop/computer, and physical robot. Content must be dynamically adjusted to match the user's selected hardware capabilities, providing appropriate examples, code snippets, and exercises for each platform. The PersonalizationToggle component must allow users to switch between personalized and original content at any time. Personalization must be context-aware, ensuring that simulation-based content is shown to laptop users, mobile-specific content to mobile users, and real-world robotics examples to physical robot users.

### Multilingual Content Support & Urdu Translation
All content must support multilingual translation capabilities, with priority for Urdu language support. The translation service must provide both English-to-Urdu and Urdu-to-English translation functionality with proper caching and fallback mechanisms. The translation system must preserve document structure and heading tags while translating main content areas. Sidebar navigation and other UI elements must also be translatable while maintaining the original functionality. The system must allow users to toggle between languages using a translation toggle, with proper state management to remember user preferences. Translation must be applied dynamically to main content areas while preserving code blocks, technical diagrams, and other non-translatable elements.

### Content Personalization from Markdown Files
All content rendered from Markdown files must support dynamic personalization through JavaScript or agent-controlled functionality. The system must convert Markdown content to personalized HTML based on user preferences, with the ability to toggle personalization on/off and reset to original content. Personalization logic must operate on the full Markdown content before rendering, ensuring that all content sections can be modified according to user settings. The personalization system must maintain the original content structure while applying transformations based on user profile data and preferences.

### Chatbot Conversation Persistence
The chatbot system must implement conversation persistence using either user session storage or localStorage to maintain conversation history across page refreshes. All conversation data must be preserved when users navigate between pages or refresh the browser, ensuring continuity of context and conversation flow. The system must implement proper cleanup mechanisms to prevent storage overflow and handle cases where storage is unavailable or full. Conversation history must be securely stored and accessible only to the authenticated user, with appropriate encryption for sensitive information.

### Full-Page Text Highlighting & Content Selection
The TextHighlighter functionality must operate on the entire page content, not just the first 10-15 lines. The system must implement proper content selection mechanisms that allow users to highlight and extract text from any part of the page, including dynamically loaded content. The highlighting functionality must integrate seamlessly with the chatbot system to ensure that selected content is properly sent for analysis and response generation. All content extraction must preserve formatting and structure to ensure accurate processing by the AI agent.

### Unified Search Functionality
All search functionality must provide consistent behavior across the application, with CustomSearch.js implementing the same search capabilities as SearchModal.js. The search system must return proper results without 404 errors and integrate with the backend content retrieval system. Search functionality must support full-text search across all content types, with proper error handling and fallback mechanisms. The search system must maintain consistent UI/UX patterns and provide equivalent functionality regardless of which search component is used.

## Technical Requirements

Technology stack requirements: Python 3.8+, ROS 2 Humble Hawksbill, Gazebo Harmonic, Unity 2022.3 LTS, NVIDIA Isaac Sim, better-auth with MCP server, PostgreSQL for user data and preferences, Qdrant for vector embeddings, OpenAI API for RAG functionality. All code examples must be compatible with these platforms. Hardware requirements: Content must be applicable to both simulation and physical robot platforms including but not limited to humanoid platforms like NAO, Pepper, Atlas, and custom builds. Compliance standards: All content must adhere to IEEE standards for robotics and AI safety protocols, as well as security best practices for user authentication and data protection.

## Development & Review Process

Content development workflow: Each section requires technical review by domain experts, code review for all implementation examples, and pedagogical review for educational effectiveness. Quality gates: All content must pass unit testing of code examples, integration testing in simulation environments, and peer review validation. Deployment approval: Content must be validated in educational settings before final publication.

## Governance

This constitution supersedes all other practices and guidelines for the Physical AI & Humanoid Robotics textbook project. All PRs/reviews must verify compliance with these principles. Amendments to this constitution require documentation of rationale, approval by the editorial board, and a migration plan for existing content. Complexity must be justified with clear educational value. Use the textbook development guidelines for runtime development guidance.

**Version**: 1.4.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2026-01-02