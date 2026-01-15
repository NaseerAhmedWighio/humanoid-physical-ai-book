/**
 * Content Personalization Service
 * Manages hardware-specific content personalization based on user preferences
 */

class ContentPersonalizationService {
  constructor() {
    this.hardwareMappings = {
      physical_robot: {
        keywords: ['physical', 'real', 'robot', 'hardware', 'calibration', 'testing'],
        contentTypes: ['hardware-specific', 'real-world', 'calibration', 'physical-testing'],
        exclusions: ['simulation', 'virtual', 'gazebo']
      },
      laptop: {
        keywords: ['simulation', 'virtual', 'gazebo', 'environment', 'launch', 'testing'],
        contentTypes: ['simulation', 'virtual', 'gazebo', 'launch-files'],
        exclusions: ['physical', 'real-world', 'hardware']
      },
      mobile: {
        keywords: ['mobile', 'remote', 'control', 'interface', 'app', 'mobile-app'],
        contentTypes: ['mobile', 'remote-control', 'mobile-apps', 'interfaces'],
        exclusions: ['physical', 'simulation']
      }
    };
  }

  /**
   * Personalize content based on user's hardware preferences
   * @param {Object} content - Original content object
   * @param {Object} userPreferences - User's hardware preferences
   * @returns {Object} - Personalized content
   */
  personalizeContent(content, userPreferences) {
    if (!userPreferences || !userPreferences.personalization_enabled) {
      return content; // Return original content if personalization is disabled
    }

    // Determine which hardware-specific content to show based on user preferences
    const activeHardware = this.getActiveHardware(userPreferences);

    if (activeHardware.length === 0) {
      return content; // Return original if no specific hardware selected
    }

    // Create a copy of the content to avoid modifying the original
    const personalizedContent = { ...content };

    // Apply personalization based on active hardware
    for (const hardware of activeHardware) {
      if (this.hardwareMappings[hardware]) {
        personalizedContent.content = this.applyHardwarePersonalization(
          content.content || '',
          hardware,
          userPreferences
        );
      }
    }

    return personalizedContent;
  }

  /**
   * Get active hardware based on user preferences
   * @param {Object} userPreferences
   * @returns {Array} - Array of active hardware types
   */
  getActiveHardware(userPreferences) {
    const active = [];

    if (userPreferences.has_physical_robot) {
      active.push('physical_robot');
    }
    if (userPreferences.has_laptop) {
      active.push('laptop');
    }
    if (userPreferences.has_mobile) {
      active.push('mobile');
    }

    return active;
  }

  /**
   * Apply hardware-specific personalization to content
   * @param {string} content
   * @param {string} hardwareType
   * @param {Object} userPreferences
   * @returns {string} - Personalized content
   */
  applyHardwarePersonalization(content, hardwareType, userPreferences) {
    if (!content || typeof content !== 'string') {
      return content;
    }

    let personalizedContent = content;

    // Apply specific personalization based on hardware type
    switch (hardwareType) {
      case 'physical_robot':
        personalizedContent = this.personalizeForPhysicalRobot(personalizedContent, userPreferences);
        break;
      case 'laptop':
        personalizedContent = this.personalizeForSimulation(personalizedContent, userPreferences);
        break;
      case 'mobile':
        personalizedContent = this.personalizeForMobile(personalizedContent, userPreferences);
        break;
      default:
        break;
    }

    return personalizedContent;
  }

  /**
   * Personalize content for physical robot users
   */
  personalizeForPhysicalRobot(content, userPreferences) {
    // Add physical robot specific notes and examples
    const physicalRobotNotes = `
> **Physical Robot Note:** This content is specifically designed for users with physical robots.
> You can test these examples on your real hardware for optimal learning experience.
> Remember to follow safety protocols when working with physical robots.
    `;

    // Replace generic content with physical robot specific content
    let updatedContent = content;
    updatedContent = updatedContent.replace(/(simulation|virtual)/gi, 'physical robot');
    updatedContent = updatedContent.replace(/(Gazebo|virtual environment)/gi, 'physical robot environment');

    // Add physical robot notes to relevant sections
    updatedContent = updatedContent.replace(
      /(##\s+.*\n)/g,
      (match) => `${match}${physicalRobotNotes}\n`
    );

    return updatedContent;
  }

  /**
   * Personalize content for simulation users
   */
  personalizeForSimulation(content, userPreferences) {
    // Add simulation specific notes and examples
    const simulationNotes = `
> **Simulation Note:** This content is optimized for simulation environments.
> You can run these examples in Gazebo or other simulation tools without requiring physical hardware.
    `;

    // Replace content to emphasize simulation
    let updatedContent = content;
    updatedContent = updatedContent.replace(/(physical robot|real hardware)/gi, 'simulation environment');
    updatedContent = updatedContent.replace(/(safely test|test without risk)/gi, 'test in simulation');

    // Add simulation notes to relevant sections
    updatedContent = updatedContent.replace(
      /(##\s+.*\n)/g,
      (match) => `${match}${simulationNotes}\n`
    );

    return updatedContent;
  }

  /**
   * Personalize content for mobile users
   */
  personalizeForMobile(content, userPreferences) {
    // Add mobile specific notes and examples
    const mobileNotes = `
> **Mobile Development Note:** This content includes mobile-specific examples and considerations.
> You can use your mobile device to control and monitor your robot remotely.
    `;

    // Replace content to emphasize mobile aspects
    let updatedContent = content;
    updatedContent = updatedContent.replace(/(desktop|laptop)/gi, 'mobile device');
    updatedContent = updatedContent.replace(/(local development)/gi, 'remote mobile development');

    // Add mobile notes to relevant sections
    updatedContent = updatedContent.replace(
      /(##\s+.*\n)/g,
      (match) => `${match}${mobileNotes}\n`
    );

    return updatedContent;
  }

  /**
   * Get personalization summary for user
   */
  getPersonalizationSummary(userPreferences) {
    if (!userPreferences) {
      return { enabled: false, summary: 'Personalization disabled' };
    }

    const activeHardware = this.getActiveHardware(userPreferences);
    const hasOtherHardware = userPreferences.has_other_hardware;
    const experienceLevel = userPreferences.web_dev_experience;

    const summary = {
      enabled: userPreferences.personalization_enabled,
      activeHardware,
      hasOtherHardware,
      experienceLevel,
      personalizationType: activeHardware.length > 0
        ? activeHardware.join(', ')
        : 'general'
    };

    return summary;
  }

  /**
   * Check if content should be shown based on user preferences
   */
  shouldShowContent(contentType, userPreferences) {
    if (!userPreferences || !userPreferences.personalization_enabled) {
      return true; // Show all content if personalization is disabled
    }

    const activeHardware = this.getActiveHardware(userPreferences);

    // If no specific hardware is selected, show general content
    if (activeHardware.length === 0) {
      return !contentType || !['hardware-specific', 'simulation', 'mobile'].includes(contentType);
    }

    // Show content that matches user's hardware
    for (const hardware of activeHardware) {
      if (this.hardwareMappings[hardware].contentTypes.includes(contentType)) {
        return true;
      }
    }

    // Show general content for users with specific hardware
    return !contentType || contentType === 'general';
  }

  /**
   * Get personalized navigation options
   */
  getPersonalizedNavigation(userPreferences) {
    if (!userPreferences || !userPreferences.personalization_enabled) {
      return null; // Return null to use default navigation
    }

    const activeHardware = this.getActiveHardware(userPreferences);
    const navOptions = [];

    if (activeHardware.includes('physical_robot')) {
      navOptions.push({
        title: 'Physical Robot Tutorials',
        path: '/docs/physical-robot',
        priority: 1
      });
    }

    if (activeHardware.includes('laptop')) {
      navOptions.push({
        title: 'Simulation Tutorials',
        path: '/docs/simulation',
        priority: 1
      });
    }

    if (activeHardware.includes('mobile')) {
      navOptions.push({
        title: 'Mobile Control',
        path: '/docs/mobile',
        priority: 1
      });
    }

    return navOptions;
  }
}

// Create and export a singleton instance
const contentPersonalizationService = new ContentPersonalizationService();
export default contentPersonalizationService;