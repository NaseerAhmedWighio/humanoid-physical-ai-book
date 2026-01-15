/**
 * Service for handling full-page content selection
 */

/**
 * Gets the currently selected text from the page
 * @returns {string} The selected text
 */
export const getSelectedText = () => {
  const selection = window.getSelection ? window.getSelection() : null;
  return selection ? selection.toString().trim() : '';
};

/**
 * Gets detailed information about the current text selection
 * @returns {Object|null} Selection details or null if no selection
 */
export const getSelectionDetails = () => {
  const selection = window.getSelection ? window.getSelection() : null;

  if (!selection || selection.toString().trim() === '') {
    return null;
  }

  const range = selection.rangeCount > 0 ? selection.getRangeAt(0) : null;

  if (!range) {
    return null;
  }

  return {
    text: selection.toString().trim(),
    range: range,
    rect: range.getBoundingClientRect(),
    startContainer: range.startContainer,
    startOffset: range.startOffset,
    endContainer: range.endContainer,
    endOffset: range.endOffset,
    commonAncestor: range.commonAncestorContainer
  };
};

/**
 * Selects all text within a specific element
 * @param {Element} element - The element to select text from
 * @returns {string} The selected text
 */
export const selectAllTextInElement = (element) => {
  if (!element) return '';

  const selection = window.getSelection();
  if (!selection) return '';

  selection.removeAllRanges();

  const range = document.createRange();
  range.selectNodeContents(element);
  selection.addRange(range);

  return selection.toString().trim();
};

/**
 * Selects all text on the entire page
 * @returns {string} The selected text
 */
export const selectAllPageText = () => {
  const selection = window.getSelection();
  if (!selection) return '';

  selection.removeAllRanges();

  const range = document.createRange();
  range.selectNodeContents(document.body);
  selection.addRange(range);

  // Limit the selection to main content areas to avoid selecting UI elements
  const mainContent = document.querySelector('main, .main-content, .content, [role="main"]') || document.body;
  if (mainContent) {
    range.selectNodeContents(mainContent);
    selection.removeAllRanges();
    selection.addRange(range);
  }

  return selection.toString().trim();
};

/**
 * Gets text from a specific section of the page
 * @param {string} selector - CSS selector for the section
 * @returns {string} The text content of the section
 */
export const getTextFromSection = (selector) => {
  const element = document.querySelector(selector);
  return element ? element.textContent.trim() : '';
};

/**
 * Gets text from the current viewport (visible area)
 * @returns {string} The text content in the viewport
 */
export const getTextFromViewport = () => {
  const elements = document.elementsFromPoint(
    window.innerWidth / 2,
    window.innerHeight / 2
  );

  // Get text from elements in the center of the viewport
  let viewportText = '';
  elements.forEach(element => {
    if (element.textContent) {
      viewportText += element.textContent.trim() + ' ';
    }
  });

  return viewportText.trim();
};

/**
 * Clears the current text selection
 */
export const clearSelection = () => {
  const selection = window.getSelection();
  if (selection) {
    selection.removeAllRanges();
  }
};

/**
 * Highlights selected text with a temporary visual indicator
 * @param {string} color - The highlight color (default: yellow)
 */
export const highlightSelection = (color = '#ffff0055') => {
  const selection = window.getSelection();
  if (!selection || selection.toString().trim() === '') {
    return;
  }

  const range = selection.getRangeAt(0);
  const span = document.createElement('span');
  span.style.backgroundColor = color;
  span.style.transition = 'background-color 0.5s ease';

  range.surroundContents(span);

  // Remove highlight after a delay
  setTimeout(() => {
    const parent = span.parentNode;
    parent.replaceChild(span.firstChild, span);
    parent.normalize();
  }, 2000);
};

/**
 * Gets the position of the current selection
 * @returns {Object|null} Position object with x, y coordinates or null if no selection
 */
export const getSelectionPosition = () => {
  const selection = window.getSelection();
  if (!selection || selection.toString().trim() === '') {
    return null;
  }

  if (selection.rangeCount > 0) {
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();
    return {
      x: rect.left + window.scrollX,
      y: rect.top + window.scrollY,
      width: rect.width,
      height: rect.height
    };
  }

  return null;
};

/**
 * Selects text by coordinates (useful for programmatic selection)
 * @param {number} x - X coordinate
 * @param {number} y - Y coordinate
 * @param {number} width - Width of selection area
 * @param {number} height - Height of selection area
 */
export const selectTextByCoordinates = (x, y, width, height) => {
  // This is a simplified implementation
  // A full implementation would require more complex logic to identify text elements
  // within the specified coordinates
  const elements = document.elementsFromPoint(x, y);
  if (elements.length > 0) {
    const element = elements[0];
    if (element.textContent) {
      // Select the entire text content of the element at the coordinates
      const selection = window.getSelection();
      if (selection) {
        selection.removeAllRanges();
        const range = document.createRange();
        range.selectNodeContents(element);
        selection.addRange(range);
        return selection.toString().trim();
      }
    }
  }
  return '';
};

// Export default object with all functions
const ContentSelectionService = {
  getSelectedText,
  getSelectionDetails,
  selectAllTextInElement,
  selectAllPageText,
  getTextFromSection,
  getTextFromViewport,
  clearSelection,
  highlightSelection,
  getSelectionPosition,
  selectTextByCoordinates
};

export default ContentSelectionService;