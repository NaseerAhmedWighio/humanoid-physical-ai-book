import React, { useEffect, useState } from 'react';
import { usePersonalization } from '../../context/PersonalizationContext';
import { useBetterAuth } from '../../context/BetterAuthContext';
import contentPersonalizationService from '../../services/contentPersonalizationService';
import { convertMarkdownToHtml } from '../../hooks/usePersonalizedContent';

/**
 * PersonalizedContent Component
 * Transforms markdown content based on user preferences and hardware
 */
const PersonalizedContent = ({ children, content = null, markdown = false }) => {
  const { isPersonalized, userPreferences } = usePersonalization();
  const { user } = useBetterAuth();
  const [processedContent, setProcessedContent] = useState(children);

  useEffect(() => {
    if (!isPersonalized || !user) {
      // If personalization is off or no user, show original content
      setProcessedContent(children);
      return;
    }

    // If we have specific content to process
    if (content) {
      const personalized = contentPersonalizationService.personalizeContent(
        { content },
        userPreferences
      );
      setProcessedContent(personalized.content);
    } else {
      // If no specific content provided, just show original
      setProcessedContent(children);
    }
  }, [isPersonalized, user, userPreferences, content, children]);

  // If personalization is off, return original content as-is
  if (!isPersonalized || !user) {
    return <>{children}</>;
  }

  // If processed content is a string (markdown), convert to HTML
  if (typeof processedContent === 'string') {
    // Use the hook's markdown to HTML conversion function
    const htmlContent = convertMarkdownToHtml(processedContent);
    return <div dangerouslySetInnerHTML={{ __html: htmlContent }} />;
  }

  // Otherwise return the processed content
  return processedContent;
};

export default PersonalizedContent;