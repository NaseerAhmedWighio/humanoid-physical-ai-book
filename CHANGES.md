# Changes Made to Humanoid AI Textbook Website

## Date: December 10, 2025

## Overview
This document records the changes made to enhance the homepage with glowing module cards, improved styling, and other UI improvements.

## Files Modified

### 1. `frontend/src/components/HomepageFeatures.module.css`
- Enhanced module cards with glowing effect using CSS gradients
- Added animated glow pulse effect with `@keyframes glow-pulse`
- Implemented dual-layer shine effect using `::before` and `::after` pseudo-elements
- Improved hover animations with `transform: translateY(-8px) scale(1.02)`
- Enhanced box-shadow effects for better depth and visual appeal
- Increased card border radius for smoother appearance
- Added cubic-bezier transition for smoother animations

### 2. `frontend/src/components/HomepageFeatures.js`
- Updated module card structure to use the new CSS classes
- Fixed class name assignment for module cards to apply enhanced styling

### 3. `frontend/src/pages/index.js`
- Fixed incorrect CSS module import that was causing build errors
- Removed invalid `styles.heroBanner` reference that was causing build failures
- Corrected the header class assignment to use standard CSS classes

## Features Implemented

### Enhanced Module Cards
- **Glow Effect**: Cards now have a continuous glowing animation with color gradient
- **Shine Effect**: Hover triggers a dynamic shine effect that moves across the card
- **Improved Interactions**: Enhanced hover animations with scale and translation
- **Better Visual Hierarchy**: Improved depth perception with layered shadows

### Search Functionality
- Ctrl+K keyboard shortcut for quick search access
- Already existed in the codebase (Navbar.js and SearchModal.js)

### Week Tabs
- Functional tabs for navigating different course weeks
- Already existed in the codebase (in HomepageFeatures.js)

### Hero Section
- Horizontal layout with image and text content
- Already existed in the codebase (in index.js)

## Technical Details

### CSS Animations
- `glow-pulse` animation: Pulsates the glow effect between 0.4 and 0.8 opacity
- Smooth transitions using `cubic-bezier(0.23, 1, 0.32, 1)` for natural motion
- Dual-layer effect using gradient borders and shine overlays

### Responsive Design
- Maintains existing responsive behavior for different screen sizes
- Cards adapt from 4-column to 2-column to 1-column layout

## Build Process
- Successfully tested with `npm run build`
- No build errors after fixes
- Production build generates successfully

## Development Server
- Fixed webpack cache warnings by running `npx docusaurus clear`
- Development server runs successfully with `npm run start`
- Cache clearing resolves persistent cache corruption issues

## Additional Fixes
- Reverted incorrect Tailwind CSS classes to maintain Docusaurus compatibility
- Fixed layout issue in hero section by using proper Docusaurus grid classes (`col col--6`)
- Ensured build process continues to work after layout corrections

## Dependencies
- Uses existing Docusaurus framework
- Leverages React and CSS modules
- Maintains compatibility with existing codebase structure

## Recent Changes - Auth Context Fix (December 18, 2025)

### Problem Fixed
Fixed "useAuth must be used within an AuthProvider" error that was occurring in PersonalizationProvider component.

### Root Cause
The application had two different authentication contexts:
- `AuthContext.js` (traditional axios-based auth with `useAuth` hook)
- `BetterAuthContext.js` (better-auth library based auth with `useBetterAuth` hook)

The Layout was using `BetterAuthProvider` but components like `PersonalizationProvider` and `ProfileSetup` were still trying to use the old `useAuth` hook from `AuthContext.js`.

### Solution Implemented
Updated components to consistently use the `BetterAuthContext` which is properly provided by the `BetterAuthProvider` in the Layout:

#### 1. Updated PersonalizationContext.js
- Changed import from `useAuth` to `useBetterAuth`
- Changed function call from `useAuth()` to `useBetterAuth()`

#### 2. Updated ProfileSetup/index.js
- Changed import from `useAuth` to `useBetterAuth`
- Changed function call from `useAuth()` to `useBetterAuth()`

### Result
- Fixed the context mismatch error
- Ensured all authentication-related components use the same authentication context
- Maintained consistency across the application

## Recent Changes - Navbar Item Registration Fix (December 18, 2025)

### Problem Fixed
Fixed "No NavbarItem component found for type 'custom-signup-button'" error that was occurring during navbar rendering.

### Root Cause
The docusaurus.config.js file had a navbar item with `type: 'custom-signup-button'` but the ComponentTypes.js file didn't have a registration mapping for this type. Docusaurus couldn't find the corresponding component to render.

### Solution Implemented
Updated ComponentTypes.js to properly register the custom signup button component:

#### 1. Updated ComponentTypes.js
- Added import for `NavbarItemCustomSignupButton` component
- Added registration mapping `'custom-signup-button': NavbarItemCustomSignupButton` to the export object

### Result
- The custom signup button component is now properly registered
- Docusaurus can find and render the component when `type: 'custom-signup-button'` is used
- The navbar error is resolved

## Recent Changes - Router Hook Fix (December 18, 2025)

### Problem Fixed
Fixed "(0 , _docusaurus_router__WEBPACK_IMPORTED_MODULE_2__.useNavigate) is not a function" error occurring in auth components (Signup, Signin, Purpose Selection).

### Root Cause
The `useNavigate` hook was not available in the component context, likely because the components were being used in a way that doesn't have access to the React Router context.

### Solution Implemented
Replaced React Router hooks with standard browser navigation in auth components:

#### 1. Updated Signup component (frontend/src/components/Auth/Signup.js)
- Removed `useNavigate` import and hook usage
- Replaced `navigate('/auth/purpose-selection')` with `window.location.href = '/auth/purpose-selection'`

#### 2. Updated Signin component (frontend/src/components/Auth/Signin.js)
- Removed `useNavigate` import and hook usage
- Replaced `navigate('/')` with `window.location.href = '/'`

#### 3. Updated Purpose Selection page (frontend/src/pages/auth/purpose-selection.js)
- Removed `useNavigate` import and hook usage
- Replaced `navigate('/')` with `window.location.href = '/'`

### Result
- The "useNavigate is not a function" error is resolved
- All navigation functionality is preserved using standard browser APIs
- The components can now be used without requiring router context access

## Recent Changes - Auth Registration and Navigation Fixes (December 18, 2025)

### Problems Fixed
1. Fixed "Cannot read properties of undefined (reading 'email')" error during registration
2. Fixed broken navigation links between auth pages
3. Fixed signup button styling in navbar
4. Improved auth page styling to match Docusaurus theme

### Solutions Implemented

#### 1. Fixed Registration Error Handling
- Added proper error checking for user object in registration results
- Added conditional redirect logic to check if user.email exists before redirecting
- Added debug logging to help troubleshoot registration issues

#### 2. Fixed Navigation Links
- Updated links in auth components to use correct paths:
  - Changed `/signin` to `/auth/signin` in Signup component
  - Changed `/signup` to `/auth/signup` in Signin component

#### 3. Updated Navbar Signup Button Styling
- Created proper CSS for NavbarItemCustomSignupButton to match search button size
- Added consistent styling with proper padding, font-size, and hover effects
- Applied Docusaurus theme variables for better consistency

#### 4. Enhanced Auth Page Theming
- Updated auth.css to use Docusaurus theme variables for both dark and light modes:
  - Used `var(--ifm-background-color)` for form backgrounds
  - Used `var(--ifm-color-primary)` for buttons and accents
  - Used `var(--ifm-color-emphasis-*` for text colors
  - Added proper focus states and transitions
  - Added support for form inputs, checkboxes, and other elements

### Result
- Registration flow works properly with better error handling
- Navigation between auth pages works correctly
- Signup button in navbar matches search button styling
- Auth pages have consistent styling that matches Docusaurus theme in both dark and light modes

## Recent Changes - Backend API Integration Fix (December 18, 2025)

### Problem Fixed
Fixed "Cannot read properties of undefined (reading 'email')" error during registration by properly integrating with the backend API.

### Root Cause
The BetterAuthContext was using the `better-auth` library for registration, but this library was not properly configured to work with our custom FastAPI backend at `/v1/auth`. The `better-auth` library has its own endpoints and doesn't integrate with our custom backend API.

### Solution Implemented
Completely rewrote the BetterAuthContext to use our backend API directly:

#### 1. Updated BetterAuthContext.js to use direct API calls:
- **register function**: Now calls `POST /v1/auth/register` directly instead of `signUp.email()`
- **login function**: Now calls `POST /v1/auth/login` and `GET /v1/auth/me` instead of `signIn.email()`
- **logout function**: Now manages JWT tokens in localStorage directly
- **updateUserPreferences function**: Now calls `POST /v1/auth/update-preferences` directly
- **Session checking**: Now uses stored JWT token to fetch user data from `GET /v1/auth/me`

#### 2. Added proper token management:
- Store JWT tokens in localStorage after successful login
- Include Authorization headers for protected API calls
- Clear tokens on logout
- Check session status on initial load using stored token

#### 3. Removed dependency on better-auth for auth operations:
- Removed `signUp`, `signIn`, and `signOut` imports from auth service
- Implemented all auth operations using fetch API calls to our backend

### Result
- Registration now properly returns user object with email from backend API
- Login and other auth operations work correctly with our backend
- The "Cannot read properties of undefined (reading 'email')" error is resolved
- Full integration with our custom FastAPI authentication system

## Recent Changes - Password Validation and Bcrypt Limitation Fix (December 18, 2025)

### Problem Fixed
Fixed "password cannot be longer than 72 bytes" error and enforced strong password requirements.

### Root Cause
The bcrypt hashing library has a limitation where passwords longer than 72 bytes are truncated, causing registration failures. Additionally, there was no client-side password validation to enforce strong passwords.

### Solution Implemented
Added comprehensive password validation to both auth components:

#### 1. Updated Signup component with strong password validation:
- Minimum 8 characters length requirement
- Maximum 70 characters to stay under bcrypt limit (conservative approach)
- At least one uppercase letter requirement
- At least one lowercase letter requirement
- At least one number requirement
- At least one special character requirement (!@#$%^&*()_+-=[]{}|;:,.<>?)
- Client-side validation before submitting to backend

#### 2. Updated Signin component with length validation:
- Added password length check (max 70 characters) to prevent bcrypt issues during login
- Early validation to provide immediate feedback to users

#### 3. Validation functions include:
- Comprehensive regex checks for password complexity
- Clear error messages for each validation failure
- Proper error handling and user feedback

### Result
- Passwords now meet strong security requirements
- Bcrypt limitation issues are prevented with length validation
- Users receive clear feedback about password requirements
- Registration and login work reliably with proper password validation

## Recent Changes - Backend Password Truncation Fix (December 18, 2025)

### Problem Fixed
Fixed "password cannot be longer than 72 bytes, truncate manually if necessary" error by improving backend password handling.

### Root Cause
The original backend password truncation logic was breaking UTF-8 characters when truncating passwords longer than 72 bytes, causing bcrypt hashing failures. The original approach of slicing encoded bytes and then decoding could create invalid UTF-8 sequences.

### Solution Implemented
Enhanced backend password handling in auth_service.py:

#### 1. Improved password truncation algorithm:
- Replaced byte-slicing approach with character-by-character truncation
- Added proper UTF-8 character boundary handling
- Ensured truncated passwords maintain valid UTF-8 encoding

#### 2. Added Pydantic field validators:
- Added `@field_validator` for password length in `UserCreate` model
- Added `@field_validator` for password length in `UserLogin` model
- Validates both maximum byte length (72 bytes) and minimum character length (8 characters)
- Provides clear error messages for validation failures

#### 3. Enhanced error handling:
- Proper validation occurs before bcrypt hashing
- Clear error messages returned to frontend
- Maintains data integrity during truncation

### Result
- Passwords longer than 72 bytes are properly truncated without breaking UTF-8 characters
- Registration and login work reliably with the bcrypt 72-byte limit
- Clear validation errors are provided to users when passwords are too long
- Full backend validation ensures data integrity