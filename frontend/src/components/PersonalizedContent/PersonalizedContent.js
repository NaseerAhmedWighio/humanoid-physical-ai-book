import React from 'react';
import { usePersonalization } from '../../context/PersonalizationContext';
import { useBetterAuth } from '../../context/BetterAuthContext';
import { convertMarkdownToHtml } from '../../hooks/usePersonalizedContent';

/**
 * PersonalizedContent Component
 * Can be used in MDX files to wrap content that should be personalized
 */
const PersonalizedContent = ({ children, markdown = false }) => {
  const { isPersonalized } = usePersonalization();
  const { user } = useBetterAuth();

  // If personalization is off or no user, return original content
  if (!isPersonalized || !user) {
    return <>{children}</>;
  }

  // If markdown prop is true, convert the children to HTML
  if (markdown && typeof children === 'string') {
    const htmlContent = convertMarkdownToHtml(children);
    return <div dangerouslySetInnerHTML={{ __html: htmlContent }} />;
  }

  return <>{children}</>;
};

export default PersonalizedContent;