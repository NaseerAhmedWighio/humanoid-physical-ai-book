# Authentication API Contract

**Feature**: 002-auth-ai-functionality
**Date**: 2025-12-25

## Overview

This document defines the API contracts for authentication functionality with proper error handling to prevent "undefined property" access errors.

## Endpoints

### POST /v1/auth/register
Register a new user account

**Request:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!",
  "has_mobile": true,
  "has_laptop": true,
  "has_physical_robot": false,
  "has_other_hardware": "Custom setup",
  "web_dev_experience": "intermediate"
}
```

**Response (Success):**
```json
{
  "success": true,
  "user": {
    "id": "uuid-string",
    "email": "user@example.com",
    "has_mobile": true,
    "has_laptop": true,
    "has_physical_robot": false,
    "has_other_hardware": "Custom setup",
    "web_dev_experience": "intermediate",
    "created_at": "2023-12-25T10:30:00Z"
  },
  "error": null
}
```

**Response (Error):**
```json
{
  "success": false,
  "user": null,
  "error": "User already exists"
}
```

### POST /v1/auth/login
Authenticate a user

**Request:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!"
}
```

**Response (Success):**
```json
{
  "success": true,
  "user": {
    "id": "uuid-string",
    "email": "user@example.com",
    "has_mobile": true,
    "has_laptop": true,
    "has_physical_robot": false,
    "has_other_hardware": "Custom setup",
    "web_dev_experience": "intermediate",
    "created_at": "2023-12-25T10:30:00Z"
  },
  "error": null
}
```

**Response (Error):**
```json
{
  "success": false,
  "user": null,
  "error": "Invalid credentials"
}
```

### GET /v1/auth/me
Get current user information

**Headers:**
```
Authorization: Bearer {token}
```

**Response (Success):**
```json
{
  "id": "uuid-string",
  "email": "user@example.com",
  "has_mobile": true,
  "has_laptop": true,
  "has_physical_robot": false,
  "has_other_hardware": "Custom setup",
  "web_dev_experience": "intermediate",
  "created_at": "2023-12-25T10:30:00Z"
}
```

**Response (Error):**
```json
{
  "error": "Unauthorized"
}
```

### POST /v1/auth/update-preferences
Update user preferences

**Headers:**
```
Authorization: Bearer {token}
```

**Request:**
```json
{
  "has_mobile": true,
  "has_laptop": true,
  "has_physical_robot": true,
  "has_other_hardware": "NAO Robot",
  "web_dev_experience": "advanced"
}
```

**Response (Success):**
```json
{
  "success": true,
  "user": {
    "id": "uuid-string",
    "email": "user@example.com",
    "has_mobile": true,
    "has_laptop": true,
    "has_physical_robot": true,
    "has_other_hardware": "NAO Robot",
    "web_dev_experience": "advanced",
    "created_at": "2023-12-25T10:30:00Z",
    "updated_at": "2023-12-25T11:00:00Z"
  },
  "error": null
}
```

### POST /v1/better-auth/register
Better Auth compatible registration

**Request:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!"
}
```

**Response (Success):**
```json
{
  "user": {
    "id": "uuid-string",
    "email": "user@example.com",
    "has_mobile": false,
    "has_laptop": false,
    "has_physical_robot": false,
    "has_other_hardware": null,
    "web_dev_experience": null,
    "created_at": "2023-12-25T10:30:00Z"
  },
  "session": {
    "access_token": "jwt-token-string",
    "token_type": "bearer"
  }
}
```

### POST /v1/better-auth/login
Better Auth compatible login

**Request:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!"
}
```

**Response (Success):**
```json
{
  "user": {
    "id": "uuid-string",
    "email": "user@example.com",
    "has_mobile": false,
    "has_laptop": false,
    "has_physical_robot": false,
    "has_other_hardware": null,
    "web_dev_experience": null,
    "created_at": "2023-12-25T10:30:00Z"
  },
  "session": {
    "access_token": "jwt-token-string",
    "token_type": "bearer"
  }
}
```