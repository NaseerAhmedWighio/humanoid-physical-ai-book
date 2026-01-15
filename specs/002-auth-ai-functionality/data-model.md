# Data Model: Auth and AI Functionality Updates

**Feature**: 002-auth-ai-functionality
**Date**: 2025-12-25

## Overview

This document defines the data models required for implementing secure user authentication with proper error handling and contextual AI question-answering functionality.

## Core Entities

### User
Represents a registered user in the system

**Fields:**
- `id` (UUID): Unique identifier for the user
- `email` (String, unique): User's email address
- `hashed_password` (String): Securely hashed password
- `has_mobile` (Boolean): Whether user has access to mobile hardware
- `has_laptop` (Boolean): Whether user has access to laptop hardware
- `has_physical_robot` (Boolean): Whether user has access to physical robot hardware
- `has_other_hardware` (String, nullable): Additional hardware description
- `web_dev_experience` (String, nullable): User's web development experience level
- `created_at` (DateTime): Account creation timestamp
- `updated_at` (DateTime): Last update timestamp

**Validation Rules:**
- Email must be valid email format
- Email must be unique
- Password must meet security requirements

### ChatSession
Represents a conversation session between user and AI assistant

**Fields:**
- `id` (UUID): Unique identifier for the session
- `user_id` (UUID): Reference to the user who owns this session
- `title` (String): Descriptive title for the session
- `created_at` (DateTime): Session creation timestamp
- `updated_at` (DateTime): Last interaction timestamp

**Relationships:**
- One-to-many with ChatMessage (one session can have many messages)

### ChatMessage
Represents a single message in a conversation

**Fields:**
- `id` (UUID): Unique identifier for the message
- `session_id` (UUID): Reference to the parent session
- `role` (String): Message role (user, assistant)
- `content` (Text): The actual message content
- `sources` (JSON, nullable): Source attribution for AI responses
- `created_at` (DateTime): Message creation timestamp

**Validation Rules:**
- Role must be either 'user' or 'assistant'
- Content must not be empty

### UserPreference
Stores user preferences for content personalization

**Fields:**
- `id` (UUID): Unique identifier
- `user_id` (UUID): Reference to the user
- `preference_key` (String): Key for the preference
- `preference_value` (String): Value of the preference
- `created_at` (DateTime): Creation timestamp
- `updated_at` (DateTime): Last update timestamp

**Validation Rules:**
- User ID must reference an existing user
- Preference key must be from allowed set of keys

## API Response Models

### AuthResponse
Standardized response for authentication operations

**Structure:**
```json
{
  "success": Boolean,
  "user": {
    "id": String,
    "email": String,
    "has_mobile": Boolean,
    "has_laptop": Boolean,
    "has_physical_robot": Boolean,
    "has_other_hardware": String,
    "web_dev_experience": String,
    "created_at": String
  } | null,
  "error": String | null
}
```

### ChatResponse
Response for AI chat operations

**Structure:**
```json
{
  "response": String,
  "sources": Array,
  "session_id": String,
  "message_id": String
}
```

### ErrorResponse
Standardized error response

**Structure:**
```json
{
  "error": String,
  "details": Object | null
}
```

## Database Constraints

1. **User Table:**
   - Email uniqueness constraint
   - Email format validation
   - Password length validation

2. **ChatSession Table:**
   - Foreign key constraint to User
   - Cascade delete for associated messages

3. **ChatMessage Table:**
   - Foreign key constraint to ChatSession
   - Check constraint for role field (user/assistant)

## Indexing Strategy

1. **User Table:**
   - Index on email for fast lookups
   - Index on created_at for chronological queries

2. **ChatSession Table:**
   - Index on user_id for user-specific queries
   - Index on updated_at for recent sessions

3. **ChatMessage Table:**
   - Index on session_id for session-specific queries
   - Index on created_at for chronological message retrieval