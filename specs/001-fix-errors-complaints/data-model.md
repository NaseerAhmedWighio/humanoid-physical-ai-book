# Data Model for Fix Errors and Complaints Feature

## Entities

### User Preferences
- **Fields**:
  - userId: string (reference to user account)
  - hardwarePreference: string (mobile, laptop, or physical robot)
  - personalizationEnabled: boolean (whether personalization is currently active)
  - createdAt: timestamp
  - updatedAt: timestamp

- **Validation rules**:
  - hardwarePreference must be one of: 'mobile', 'laptop', 'physical_robot'
  - userId must reference an existing user account

### Chat Conversation
- **Fields**:
  - conversationId: string (unique identifier)
  - userId: string (optional - for authenticated users)
  - messages: array of message objects
    - role: string ('user' or 'assistant')
    - content: string (message content)
    - timestamp: timestamp
  - createdAt: timestamp
  - updatedAt: timestamp

- **State transitions**:
  - New conversation created when user starts chatting
  - Messages added as conversation progresses
  - Conversation updated when messages are added

### Personalized Content
- **Fields**:
  - originalContentId: string (reference to original content)
  - userId: string (reference to user)
  - hardwarePreference: string (the target hardware for personalization)
  - personalizedContent: string (the transformed content)
  - createdAt: timestamp

- **Validation rules**:
  - hardwarePreference must be one of: 'mobile', 'laptop', 'physical_robot'

### Search Results
- **Fields**:
  - query: string (the search query)
  - results: array of content items
    - contentId: string
    - title: string
    - snippet: string
    - relevanceScore: number
  - searchTime: timestamp
  - totalResults: number

- **Validation rules**:
  - query must not be empty
  - results array can be empty but must be an array