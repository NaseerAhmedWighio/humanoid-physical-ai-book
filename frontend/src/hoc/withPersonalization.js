import React, { useEffect, useState, useCallback } from 'react';
import { usePersonalization } from '../../context/PersonalizationContext';
import { useBetterAuth } from '../../context/BetterAuthContext';
import contentPersonalizationService from '../../services/contentPersonalizationService';

/**
 * A higher-order component that wraps page content to enable personalization
 * It intercepts the page content and applies personalization based on user preferences
 */
const withPersonalization = (WrappedComponent) => {
  const WithPersonalization = (props) => {
    const { isPersonalized, userPreferences } = usePersonalization();
    const { user } = useBetterAuth();
    const [processedContent, setProcessedContent] = useState(null);

    // Function to extract text content from React elements
    const extractContentFromElements = useCallback((element) => {
      if (typeof element === 'string' || typeof element === 'number') {
        return element;
      }

      if (React.isValidElement(element)) {
        if (typeof element.type === 'string') {
          // Native DOM element
          return element.props.children ? extractContentFromElements(element.props.children) : '';
        } else {
          // Custom component - return as is
          return element;
        }
      }

      if (Array.isArray(element)) {
        return element.map(extractContentFromElements).join(' ');
      }

      return '';
    }, []);

    useEffect(() => {
      if (!isPersonalized || !user) {
        // If personalization is off or no user, use original content
        setProcessedContent(null);
        return;
      }

      // Try to extract content from the wrapped component's props
      if (props.content || props.children) {
        let contentToProcess = '';

        if (props.content) {
          contentToProcess = props.content;
        } else if (props.children) {
          contentToProcess = extractContentFromElements(props.children);
        }

        if (contentToProcess && typeof contentToProcess === 'string') {
          const personalized = contentPersonalizationService.personalizeContent(
            { content: contentToProcess },
            userPreferences
          );
          setProcessedContent(personalized.content);
        }
      }
    }, [isPersonalized, user, userPreferences, props, extractContentFromElements]);

    // If personalization is enabled and we have processed content, use it
    if (isPersonalized && processedContent && typeof processedContent === 'string') {
      // Convert markdown to HTML for personalized content
      const htmlContent = convertMarkdownToHtml(processedContent);
      return (
        <div className="personalized-content-wrapper">
          <div dangerouslySetInnerHTML={{ __html: htmlContent }} />
        </div>
      );
    }

    // Otherwise return the original component
    return <WrappedComponent {...props} />;
  };

  return WithPersonalization;
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
  html = html.replace(/^## (.*$)/gim, '<h2>$1</h2>');
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
  html = html.replace(/^\* (.*$)/gm, '<li>$1</li>');
  html = html.replace(/^\d+\. (.*$)/gm, '<li>$1</li>');

  // Wrap list items in proper tags
  html = html.replace(/(<li>.*<\/li>)/gs, '<ul>$1</ul>');
  // Note: This is a simplified approach - in a real implementation, you'd need more sophisticated list handling

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

export default withPersonalization;