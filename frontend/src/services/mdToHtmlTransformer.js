/**
 * Service for transforming Markdown content to HTML with personalization
 */

import { applyHardwarePreference } from './personalizationService';

/**
 * Converts Markdown content to HTML and applies personalization based on hardware preference
 * @param {string} markdownContent - The original Markdown content
 * @param {string} hardwarePreference - The target hardware preference
 * @returns {string} - The transformed HTML content
 */
export const transformMdToHtml = (markdownContent, hardwarePreference) => {
  // Convert markdown to HTML (simplified version)
  // In a real implementation, this would use a proper markdown parser like marked.js or markdown-it
  let htmlContent = convertMarkdownToHtml(markdownContent);

  // Apply hardware-specific personalization
  return applyHardwarePreference(htmlContent, hardwarePreference);
};

/**
 * Basic markdown to HTML conversion
 * @param {string} markdown - The markdown content to convert
 * @returns {string} - The HTML content
 */
const convertMarkdownToHtml = (markdown) => {
  if (!markdown) return '';

  // Convert headers
  let html = markdown.replace(/^### (.*$)/gim, '<h3>$1</h3>');
  html = html.replace(/^## (.*$)/gim, '<h2>$1</h2>');
  html = html.replace(/^# (.*$)/gim, '<h1>$1</h1>');

  // Convert bold and italic
  html = html.replace(/\*\*(.*?)\*\*/gim, '<strong>$1</strong>');
  html = html.replace(/\*(.*?)\*/gim, '<em>$1</em>');

  // Convert links
  html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/gim, '<a href="$2">$1</a>');

  // Convert images
  html = html.replace(/!\[([^\]]*)\]\(([^)]+)\)/gim, '<img alt="$1" src="$2" />');

  // Convert paragraphs
  html = html.replace(/\n\n/gim, '</p><p>');
  html = '<p>' + html + '</p>';
  html = html.replace(/<p><\/p>/gim, '');

  // Convert line breaks
  html = html.replace(/\n/gim, '<br />');

  // Clean up multiple paragraph tags
  html = html.replace(/(<p>)+/g, '<p>');
  html = html.replace(/(<\/p>)+/g, '</p>');

  return html;
};

/**
 * Applies personalization to markdown content based on hardware preference
 * @param {string} markdownContent - The original Markdown content
 * @param {string} hardwarePreference - The target hardware preference
 * @returns {string} - The personalized HTML content
 */
export const personalizeMarkdownContent = (markdownContent, hardwarePreference) => {
  try {
    // First convert markdown to HTML
    const htmlContent = convertMarkdownToHtml(markdownContent);

    // Then apply hardware-specific transformations
    return applyHardwarePreference(htmlContent, hardwarePreference);
  } catch (error) {
    console.error('Error personalizing markdown content:', error);
    // Return original content if transformation fails
    return markdownContent;
  }
};

/**
 * Processes markdown content with additional transformations for specific hardware types
 * @param {string} markdownContent - The original Markdown content
 * @param {string} hardwarePreference - The target hardware preference
 * @returns {string} - The processed HTML content
 */
export const processMarkdownForHardware = (markdownContent, hardwarePreference) => {
  let processedContent = markdownContent;

  switch (hardwarePreference) {
    case 'mobile':
      // Add mobile-specific transformations
      processedContent = addMobileOptimizations(processedContent);
      break;
    case 'laptop':
      // Add desktop-specific transformations
      processedContent = addDesktopOptimizations(processedContent);
      break;
    case 'physical_robot':
      // Add robot-specific transformations
      processedContent = addRobotOptimizations(processedContent);
      break;
    default:
      // No additional processing for unknown hardware types
      break;
  }

  // Convert to HTML
  return convertMarkdownToHtml(processedContent);
};

/**
 * Adds mobile-specific optimizations to markdown content
 * @param {string} markdownContent - The original Markdown content
 * @returns {string} - The optimized markdown content
 */
const addMobileOptimizations = (markdownContent) => {
  // Add mobile-friendly classes to images
  return markdownContent.replace(/!\[([^\]]*)\]\(([^)]+)\)/g, (match, alt, src) => {
    return `<div class="mobile-image-container">![${alt}](${src})</div>`;
  });
};

/**
 * Adds desktop-specific optimizations to markdown content
 * @param {string} markdownContent - The original Markdown content
 * @returns {string} - The optimized markdown content
 */
const addDesktopOptimizations = (markdownContent) => {
  // Add desktop-specific content or formatting
  return markdownContent;
};

/**
 * Adds robot-specific optimizations to markdown content
 * @param {string} markdownContent - The original Markdown content
 * @returns {string} - The optimized markdown content
 */
const addRobotOptimizations = (markdownContent) => {
  // Add robot-specific instructions or modifications
  return `<!-- Robot-specific content -->\n${markdownContent}\n<!-- End robot-specific content -->`;
};

// Export default object with all functions
const MdToHtmlTransformer = {
  transformMdToHtml,
  personalizeMarkdownContent,
  processMarkdownForHardware
};

export default MdToHtmlTransformer;