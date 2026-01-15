/**
 * Service for handling personalization logic on the frontend
 */

/**
 * Maps hardware preferences to specific content transformations
 * @param {string} content - The original content to transform
 * @param {string} hardwarePreference - The target hardware preference ('mobile', 'laptop', 'physical_robot')
 * @returns {string} - The transformed content
 */
export const applyHardwarePreference = (content, hardwarePreference) => {
  switch (hardwarePreference) {
    case 'mobile':
      return applyMobileOptimization(content);
    case 'laptop':
      return applyDesktopOptimization(content);
    case 'physical_robot':
      return applyRobotOptimization(content);
    default:
      return content; // Return original content if preference is not recognized
  }
};

/**
 * Applies mobile-specific optimizations to content
 * @param {string} content - The original content
 * @returns {string} - Mobile-optimized content
 */
const applyMobileOptimization = (content) => {
  // Add mobile-specific classes and optimizations
  let optimizedContent = content;

  // Replace large images with mobile-friendly versions
  optimizedContent = optimizedContent.replace(
    /(<img[^>]*?)width=["'](\d+)["']/gi,
    (match, imgTag, width) => {
      const numWidth = parseInt(width);
      if (numWidth > 800) {
        return imgTag + `width="${Math.min(numWidth, 600)}"`;
      }
      return match;
    }
  );

  // Add mobile-specific CSS classes
  optimizedContent = optimizedContent.replace(
    /<div/gi,
    '<div class="mobile-friendly"'
  );

  // Optimize for mobile touch interactions
  optimizedContent = optimizedContent.replace(
    /<button/gi,
    '<button class="mobile-button"'
  );

  return optimizedContent;
};

/**
 * Applies desktop-specific optimizations to content
 * @param {string} content - The original content
 * @returns {string} - Desktop-optimized content
 */
const applyDesktopOptimization = (content) => {
  // Add desktop-specific classes and optimizations
  let optimizedContent = content;

  // Add desktop-specific CSS classes
  optimizedContent = optimizedContent.replace(
    /<div/gi,
    '<div class="desktop-friendly"'
  );

  // Optimize for desktop interactions
  optimizedContent = optimizedContent.replace(
    /<button/gi,
    '<button class="desktop-button"'
  );

  return optimizedContent;
};

/**
 * Applies physical robot-specific optimizations to content
 * @param {string} content - The original content
 * @returns {string} - Robot-specific content
 */
const applyRobotOptimization = (content) => {
  // Add robot-specific content and optimizations
  let optimizedContent = content;

  // Add robot-specific classes
  optimizedContent = optimizedContent.replace(
    /<div/gi,
    '<div class="robot-content"'
  );

  // Add robot-specific instructions or modifications
  optimizedContent = `<div class="robot-instructions">
    <p>This content is optimized for physical robot interaction.</p>
  </div>` + optimizedContent;

  return optimizedContent;
};

/**
 * Gets the display name for a hardware preference
 * @param {string} hardwarePreference - The hardware preference
 * @returns {string} - The display name
 */
export const getHardwarePreferenceDisplayName = (hardwarePreference) => {
  switch (hardwarePreference) {
    case 'mobile':
      return 'Mobile Device';
    case 'laptop':
      return 'Laptop/Computer';
    case 'physical_robot':
      return 'Physical Robot';
    default:
      return 'Unknown';
  }
};

/**
 * Validates a hardware preference
 * @param {string} hardwarePreference - The hardware preference to validate
 * @returns {boolean} - True if valid, false otherwise
 */
export const isValidHardwarePreference = (hardwarePreference) => {
  return ['mobile', 'laptop', 'physical_robot'].includes(hardwarePreference);
};

/**
 * Gets the default hardware preference
 * @returns {string} - The default hardware preference
 */
export const getDefaultHardwarePreference = () => {
  return 'laptop';
};

// Export default object with all functions
const PersonalizationService = {
  applyHardwarePreference,
  getHardwarePreferenceDisplayName,
  isValidHardwarePreference,
  getDefaultHardwarePreference
};

export default PersonalizationService;