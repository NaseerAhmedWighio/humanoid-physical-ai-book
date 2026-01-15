# Chat/AI API Contract

**Feature**: 002-auth-ai-functionality
**Date**: 2025-12-25

## Overview

This document defines the API contracts for AI chat functionality with conversation context management and source attribution.

## Endpoints

### POST /v1/chat/sessions
Create a new chat session

**Headers:**
```
Authorization: Bearer {token}
```

**Request:**
```json
{
  "title": "Humanoid Robotics Discussion"
}
```

**Response (Success):**
```json
{
  "id": "uuid-string",
  "user_id": "user-uuid",
  "title": "Humanoid Robotics Discussion",
  "created_at": "2023-12-25T10:30:00Z",
  "updated_at": "2023-12-25T10:30:00Z"
}
```

### POST /v1/chat/sessions/{session_id}/messages
Send a message in a chat session

**Headers:**
```
Authorization: Bearer {token}
```

**Request:**
```json
{
  "content": "What are the key components of a humanoid robot?"
}
```

**Response (Success):**
```json
{
  "response": "A humanoid robot typically consists of several key components...",
  "sources": [
    {
      "title": "Humanoid Robotics Components",
      "url": "/docs/components",
      "content": "Detailed information about joints, actuators, sensors, etc."
    }
  ],
  "session_id": "session-uuid",
  "message_id": "message-uuid",
  "created_at": "2023-12-25T10:31:00Z"
}
```

### POST /v1/chat/ask-from-selection
Ask a question about selected text

**Headers:**
```
Authorization: Bearer {token}
```

**Request:**
```json
{
  "content": "This is the selected text that the user wants to ask about..."
}
```

**Response (Success):**
```json
{
  "response": "Based on the selected text, the concept refers to...",
  "sources": [
    {
      "title": "Related Concept Explanation",
      "url": "/docs/concepts",
      "content": "Detailed explanation of the concept in the selected text"
    }
  ],
  "session_id": "session-uuid",
  "message_id": "message-uuid"
}
```

**Response (Error):**
```json
{
  "error": "Failed to process the question",
  "details": "Error details here"
}
```

### GET /v1/chat/sessions
Get user's chat sessions

**Headers:**
```
Authorization: Bearer {token}
```

**Response (Success):**
```json
{
  "sessions": [
    {
      "id": "session-uuid-1",
      "title": "Humanoid Robotics Discussion",
      "created_at": "2023-12-25T10:30:00Z",
      "updated_at": "2023-12-25T11:00:00Z"
    },
    {
      "id": "session-uuid-2",
      "title": "ROS Integration",
      "created_at": "2023-12-24T09:15:00Z",
      "updated_at": "2023-12-24T10:45:00Z"
    }
  ]
}
```

### GET /v1/chat/sessions/{session_id}/messages
Get messages from a specific session

**Headers:**
```
Authorization: Bearer {token}
```

**Response (Success):**
```json
{
  "messages": [
    {
      "id": "message-uuid-1",
      "session_id": "session-uuid",
      "role": "user",
      "content": "What are the key components of a humanoid robot?",
      "created_at": "2023-12-25T10:30:00Z"
    },
    {
      "id": "message-uuid-2",
      "session_id": "session-uuid",
      "role": "assistant",
      "content": "A humanoid robot typically consists of several key components...",
      "sources": [
        {
          "title": "Humanoid Robotics Components",
          "url": "/docs/components",
          "content": "Detailed information about joints, actuators, sensors, etc."
        }
      ],
      "created_at": "2023-12-25T10:31:00Z"
    }
  ]
}
```