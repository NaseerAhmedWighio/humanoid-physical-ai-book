# Data Model: Content Personalization and Urdu Translation

## User Profile Entity

**Fields:**
- `id`: Unique identifier for the user
- `email`: User's email address (primary login)
- `has_mobile`: Boolean indicating if user has mobile device access
- `has_laptop`: Boolean indicating if user has laptop/computer access
- `has_physical_robot`: Boolean indicating if user has physical robot access
- `has_other_hardware`: String for additional hardware description
- `web_dev_experience`: String indicating experience level (beginner, intermediate, experienced, expert)
- `language_preference`: String for preferred language (en, ur)
- `personalization_enabled`: Boolean for personalization toggle state
- `created_at`: Timestamp for account creation
- `updated_at`: Timestamp for last update

**Validation Rules:**
- At least one hardware preference must be selected during signup
- Email must be valid format
- Experience level must be one of the predefined values
- Language preference must be supported (en, ur)

**Relationships:**
- One-to-many with user progress tracking
- One-to-many with user preferences

## Personalization Context Entity

**Fields:**
- `user_id`: Reference to the user
- `is_personalized`: Boolean indicating if personalization is currently enabled
- `active_filters`: Object containing active personalization filters
- `personalized_content`: Object containing personalized content data
- `last_updated`: Timestamp for last context update

**State Transitions:**
- `enabled` ↔ `disabled`: Toggled by user via PersonalizationToggle component

## Translation Cache Entity

**Fields:**
- `source_text`: Original text in source language
- `target_text`: Translated text in target language
- `source_lang`: Source language code (e.g., 'en')
- `target_lang`: Target language code (e.g., 'ur')
- `cache_key`: Unique key for cache lookup
- `created_at`: Timestamp for cache entry creation
- `last_accessed`: Timestamp for last access

**Validation Rules:**
- Cache entries expire after 24 hours
- Maximum cache size to prevent memory issues

## Translation Dictionary Entry

**Fields:**
- `english_word`: Original English word/phrase
- `urdu_translation`: Urdu translation
- `created_at`: Timestamp for entry creation

**Validation Rules:**
- All entries must be validated for accuracy
- Duplicates are not allowed