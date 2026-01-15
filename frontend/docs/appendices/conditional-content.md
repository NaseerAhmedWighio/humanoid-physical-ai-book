# Content Personalization and Urdu Translation Guide

This textbook features personalized content that adapts to your profile and multilingual support with Urdu translation. The content you see is customized based on:

1. **Hardware Available:**
   - Mobile device
   - Laptop/Computer
   - Physical Robot
   - Other hardware

2. **Experience Level:**
   - Beginner
   - Intermediate
   - Experienced
   - Expert

3. **Language Preference:**
   - English
   - Urdu

## Content Personalization

### How It Works

The system uses the `ConditionalContent` component to show different content based on your profile:

```markdown
<ConditionalContent forExperience={['beginner']} className="note">
### Beginner-Friendly Introduction
Content shown only to beginners...
</ConditionalContent>

<ConditionalContent forHardware={['physical_robot']} className="important">
### Physical Robot Access
Content shown only to users with physical robots...
</ConditionalContent>
```

### Personalization Toggle

The PersonalizationToggle component allows you to switch between personalized and original content. The toggle is available on all content pages except:
- Homepage
- Authentication pages (signin, signup)
- Documentation intro page

## Urdu Translation

### Translation Toggle

The TranslateButton component allows you to switch between English and Urdu content. The toggle is available in the navbar and on all content pages.

### Translation Features

- **Caching**: Translations are cached to improve performance
- **Fallback**: Dictionary-based fallback when API translation fails
- **Batch Processing**: Efficient translation of multiple text elements
- **Accessibility**: Proper ARIA attributes for screen readers
- **Preservation**: Code blocks and technical diagrams remain untranslated

### Supported Elements

The translation service translates content in:
- Main content areas
- Headings (h1-h6)
- Paragraphs (p)
- List items (li)
- Table cells (td, th)
- Sidebar navigation items

## Profile Management

You can update your preferences at any time by accessing your profile page. The profile management page allows you to update:
- Hardware preferences
- Web development experience level
- Language preferences
- Content personalization settings

## Content Classes

Different CSS classes are available for styling conditional content:

- `note` - General information
- `hint` - Helpful tips
- `warning` - Important warnings
- `important` - Critical information
- `beginner` - Beginner-specific content
- `expert` - Expert-specific content

## Validation

The system validates:
- At least one hardware preference must be selected during registration
- Language preferences must be valid (en/ur)
- Web development experience must be one of: beginner, intermediate, experienced, expert
- Password strength requirements

## Performance Optimization

The translation service includes:
- Caching with size limits (1000 entries)
- Expiration of cached translations (1 hour)
- Batch processing for efficient translation of multiple elements
- Fallback mechanisms for API failures