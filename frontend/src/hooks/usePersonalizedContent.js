import { useEffect, useState } from 'react';
import { usePersonalization } from '../context/PersonalizationContext';
import { useBetterAuth } from '../context/BetterAuthContext';
import contentPersonalizationService from '../services/contentPersonalizationService';

/**
 * Custom hook to handle content personalization
 * Can be used to transform markdown content based on user preferences
 */
export const usePersonalizedContent = (originalContent) => {
  const { isPersonalized, userPreferences } = usePersonalization();
  const { user } = useBetterAuth();
  const [personalizedContent, setPersonalizedContent] = useState(originalContent);

  useEffect(() => {
    if (!isPersonalized || !user || !originalContent) {
      setPersonalizedContent(originalContent);
      return;
    }

    // Personalize the content based on user preferences
    const personalized = contentPersonalizationService.personalizeContent(
      { content: originalContent },
      userPreferences
    );

    setPersonalizedContent(personalized.content || originalContent);
  }, [isPersonalized, user, userPreferences, originalContent]);

  return personalizedContent;
};

/**
 * Function to convert markdown to HTML
 * Handles common markdown elements for personalized content
 */
export const convertMarkdownToHtml = (markdown) => {
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
  html = html.replace(/^###### (.*$)/gm, '<h6>$1</h6>');
  html = html.replace(/^##### (.*$)/gm, '<h5>$1</h5>');
  html = html.replace(/^#### (.*$)/gm, '<h4>$1</h4>');
  html = html.replace(/^### (.*$)/gm, '<h3>$1</h3>');
  html = html.replace(/^## (.*$)/gm, '<h2>$1</h2>');
  html = html.replace(/^# (.*$)/gm, '<h1>$1</h1>');

  // Convert bold
  html = html.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');

  // Convert italic
  html = html.replace(/\*(.*?)\*/g, '<em>$1</em>');

  // Convert code blocks
  html = html.replace(/```([\s\S]*?)```/g, '<pre><code>$1</code></pre>');
  html = html.replace(/`(.*?)`/g, '<code>$1</code>');

  // Convert links
  html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank" rel="noopener">$1</a>');

  // Improved list conversion
  // First, collect all list items and wrap them properly
  const lines = html.split('\n');
  let newHtml = '';
  let inUnorderedList = false;
  let inOrderedList = false;

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    const trimmedLine = line.trim();

    // Check for unordered list item
    if (/^\s*\*\s/.test(line)) {
      const indentLevel = (line.match(/^\s*/)[0].length / 2) | 0;
      const content = line.replace(/^\s*\*\s*/, '');

      if (!inOrderedList && !inUnorderedList) {
        newHtml += '<ul>\n';
        inUnorderedList = true;
      }
      newHtml += `${'  '.repeat(indentLevel)}<li>${content}</li>\n`;
    }
    // Check for ordered list item
    else if (/^\s*\d+\.\s/.test(line)) {
      const indentLevel = (line.match(/^\s*/)[0].length / 2) | 0;
      const content = line.replace(/^\s*\d+\.\s*/, '');

      if (!inUnorderedList && !inOrderedList) {
        newHtml += '<ol>\n';
        inOrderedList = true;
      }
      newHtml += `${'  '.repeat(indentLevel)}<li>${content}</li>\n`;
    }
    // If it's not a list item, close any open lists and add the line
    else {
      if (inUnorderedList) {
        newHtml += '</ul>\n';
        inUnorderedList = false;
      }
      if (inOrderedList) {
        newHtml += '</ol>\n';
        inOrderedList = false;
      }
      newHtml += line + '\n';
    }
  }

  // Close any remaining open lists
  if (inUnorderedList) {
    newHtml += '</ul>\n';
  }
  if (inOrderedList) {
    newHtml += '</ol>\n';
  }

  html = newHtml;

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