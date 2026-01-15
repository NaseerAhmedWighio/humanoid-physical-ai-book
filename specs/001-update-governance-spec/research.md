# Research Summary: Content Personalization and Urdu Translation

## Decision: Hardware Preference Collection
**Rationale**: Users need to select their available hardware during signup (mobile, laptop, physical robot) to enable personalized content delivery. This was already implemented in the existing codebase through the Signup.js component with checkboxes for hardware preferences.

**Alternatives considered**:
- Profile page after signup - Less effective as preferences would need to be set after initial content exposure
- Automatic detection - Not feasible for hardware capabilities

## Decision: Personalization Toggle Implementation
**Rationale**: The PersonalizationToggle component already exists in the codebase and allows users to switch between personalized and original content. This component is integrated with the PersonalizationContext to manage state.

**Alternatives considered**:
- URL parameters - Would not persist user preference across sessions
- Local storage only - Less secure and not synchronized across devices

## Decision: Urdu Translation Approach
**Rationale**: The translation service already exists in the codebase (translationService.js) with English-to-Urdu and Urdu-to-English capabilities. The service uses both API-based translation with caching and fallback dictionary-based translation.

**Alternatives considered**:
- Third-party translation services only - Less reliable and potential cost concerns
- Manual translation only - Not scalable for dynamic content

## Decision: Content Structure Preservation
**Rationale**: Translation functionality preserves document structure and heading tags while translating main content areas. Code blocks, technical diagrams, and other non-translatable elements are preserved using selectors that avoid translating these elements.

**Alternatives considered**:
- Full content translation - Would damage code examples and technical diagrams
- Separate translated documents - Would be difficult to maintain and synchronize

## Decision: Toggle Placement and Exclusions
**Rationale**: Personalization toggle will be available on all pages except homepage, auth pages, and /docs/intro as specified in the requirements. This is achieved through conditional rendering in the UI components.

**Alternatives considered**:
- Universal toggle - Would clutter simple pages and auth flows
- Different exclusion rules - The specified rules align with user experience best practices