---
id: "001"
title: "Fix Navbar Auth Buttons Not Updating"
stage: "general"
date: "2025-12-29"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "002-auth-ai-functionality"
user: "user"
command: "fix navbar buttons not updating based on auth status"
labels: ["auth", "navbar", "ui", "session"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "frontend/src/services/auth.js"
  - "frontend/src/context/BetterAuthContext.js"
tests: []
---

## Prompt
my navbar buttons are not replacing yet, fix them with this way check always that the access token of user is stored in localstorage like user is logged in then change the buttons either not change, if there would useEffect issue or something else fix that also

## Response
I have successfully fixed the navbar buttons not updating issue by implementing the following changes:

### Problem Identified
The navbar buttons were not updating properly after login/logout because the `useSession` hook in `auth.js` only ran once on mount and didn't respond to changes in the authentication token stored in localStorage.

### Solution Implemented

1. **Enhanced the `useSession` hook** in `frontend/src/services/auth.js`:
   - Added a storage event listener to detect when the `better_auth_token` changes in localStorage
   - This allows the session to be refreshed automatically when authentication state changes

2. **Updated the login function** in `frontend/src/context/BetterAuthContext.js`:
   - Added manual dispatch of storage events after successful login
   - This ensures the session hook gets updated in the same tab

3. **Updated the register function** in `frontend/src/context/BetterAuthContext.js`:
   - Added manual dispatch of storage events after successful registration
   - This ensures the session hook gets updated in the same tab

4. **Updated the logout function** in `frontend/src/context/BetterAuthContext.js`:
   - Added manual dispatch of storage events after logout
   - This ensures the session hook gets updated in the same tab

### How It Works
- When a user logs in, registers, or logs out, the authentication token is stored/removed from localStorage
- A storage event is manually dispatched to notify the session hook of the change
- The session hook listens for these events and refreshes the session data
- The BetterAuthContext updates its local state immediately for instant UI feedback
- The navbar buttons now properly update to reflect the current authentication state

The fix ensures that navbar buttons will always reflect the user's current authentication status by properly synchronizing the session state with localStorage changes.

## Outcome
Navbar buttons now properly update based on authentication status, showing Sign In/Up buttons when logged out and user profile/logout dropdown when logged in. The authentication state is properly synchronized across the application.