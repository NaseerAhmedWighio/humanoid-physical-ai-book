import React, { useEffect, useState } from 'react';
import { usePersonalization } from '../../context/PersonalizationContext';
import { useBetterAuth } from '../../context/BetterAuthContext';
import contentPersonalizationService from '../../services/contentPersonalizationService';

/**
 * PageContentPersonalizer Component
 * A wrapper that can be used to personalize entire page content based on user preferences
 */
const PageContentPersonalizer = ({ children, pageContent = null }) => {
  const { isPersonalized, userPreferences } = usePersonalization();
  const { user } = useBetterAuth();
  const [personalizedContent, setPersonalizedContent] = useState(null);
  const [originalContent, setOriginalContent] = useState(null);

  useEffect(() => {
    if (!isPersonalized || !user) {
      // If personalization is off or no user, show original content
      setPersonalizedContent(null);
      return;
    }

    // If we have specific page content to process
    if (pageContent) {
      const personalized = contentPersonalizationService.personalizeContent(
        { content: pageContent },
        userPreferences
      );
      setPersonalizedContent(personalized.content);
    } else {
      // If no specific content provided, try to extract from children
      if (typeof children === 'string') {
        const personalized = contentPersonalizationService.personalizeContent(
          { content: children },
          userPreferences
        );
        setPersonalizedContent(personalized.content);
      } else {
        // For React elements, we need to extract the text content or HTML
        setOriginalContent(children);
      }
    }
  }, [isPersonalized, user, userPreferences, pageContent, children]);

  // If personalization is off or no personalized content available, return original
  if (!isPersonalized || !user || !personalizedContent) {
    return <>{children}</>;
  }

  // If personalized content is a string (markdown/html), convert and render it
  if (typeof personalizedContent === 'string') {
    // Convert markdown to HTML if needed
    const htmlContent = convertMarkdownToHtml(personalizedContent);
    return <div className="personalized-content" dangerouslySetInnerHTML={{ __html: htmlContent }} />;
  }

  // Otherwise return the personalized React elements
  return personalizedContent;
};

/**
 * Basic markdown to HTML conversion function
 * Handles common markdown elements for personalized content
 */
const convertMarkdownToHtml = (markdown) => {
  if (!markdown || typeof markdown !== 'string') {
    return markdown;
  }

  let html = markdown;

  // Escape HTML to prevent XSS
  html = html
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#x27;');

  // Convert headers
  html = html.replace(/^###### (.*$)/gim, '<h6>$1</h6>');
  html = html.replace(/^##### (.*$)/gim, '<h5>$1</h5>');
  html = html.replace(/^#### (.*$)/gim, '<h4>$1</h4>');
  html = html.replace(/^### (.*$)/gim, '<h3>$1</h3>');
  html = html.replace(/^## (.*$)/gim, '<h2>$1</h2>'); // Fixed: was $2, should be $1
  html = html.replace(/^# (.*$)/gim, '<h1>$1</h1>');

  // Convert bold
  html = html.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');

  // Convert italic
  html = html.replace(/\*(.*?)\*/g, '<em>$1</em>');

  // Convert code blocks
  html = html.replace(/```([\s\S]*?)```/g, '<pre><code>$1</code></pre>');
  html = html.replace(/`(.*?)`/g, '<code>$1</code>');

  // Convert links
  html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank" rel="noopener">$1</a>');

  // Convert lists
  html = html.replace(/^\* (.*$)/gm, '<ul><li>$1</li></ul>');
  html = html.replace(/^\d+\. (.*$)/gm, '<ol><li>$1</li></ol>');

  // Fix multiple list tags
  html = html.replace(/<\/ul>\s*<ul>/g, '');
  html = html.replace(/<\/ol>\s*<ol>/g, '');

  // Convert paragraphs (separate by double newlines)
  const paragraphs = html.split(/\n\s*\n/);
  html = paragraphs
    .map(p => {
      if (!p.trim()) return '';
      if (p.startsWith('<') && p.endsWith('>')) return p; // Already HTML
      return `<p>${p.replace(/\n/g, '<br>')}</p>`;
    })
    .join('');

  return html;
};

export default PageContentPersonalizer;