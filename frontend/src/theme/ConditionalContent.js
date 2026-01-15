import React from 'react';
import '../components/ConditionalContent/ConditionalContent.css';

/**
 * ConditionalContent component renders different content based on user's profile
 * It shows content based on hardware availability and web development experience
 */
const ConditionalContent = ({ children, forHardware = [], forExperience = [], fallback = null, className = '' }) => {
  // Since MDX components might not have access to React context in all cases,
  // we'll implement a more robust fallback mechanism
  try {
    // Attempt to import and use context hooks
    const { useTheme } = require('../context/ThemeContext');
    const { useBetterAuth } = require('../context/BetterAuthContext');
    const { usePersonalization } = require('../context/PersonalizationContext'); // Add personalization context

    let themeContext, authContext, personalizationContext;

    try {
      themeContext = useTheme();
    } catch (e) {
      console.warn('Theme context not available for ConditionalContent');
      themeContext = null;
    }

    try {
      authContext = useBetterAuth();
    } catch (e) {
      console.warn('Auth context not available for ConditionalContent');
      authContext = null;
    }

    try {
      personalizationContext = usePersonalization(); // Get personalization context
    } catch (e) {
      console.warn('Personalization context not available for ConditionalContent');
      personalizationContext = null;
    }

    // Create a profile based on authenticated user data
    const userProfile = authContext?.user ? {
      has_mobile: authContext.user.has_mobile,
      has_laptop: authContext.user.has_laptop,
      has_physical_robot: authContext.user.has_physical_robot,
      has_other_hardware: authContext.user.has_other_hardware,
      web_dev_experience: authContext.user.web_dev_experience,
      name: authContext.user.email, // Using email as name since no separate name field
      email: authContext.user.email,
      profile_picture_url: null
    } : null;

    // Check if personalization is enabled (if context is available)
    const isPersonalized = personalizationContext?.isPersonalized ?? true;

    // Check active filters
    const activeFilters = personalizationContext?.getActiveFilters?.() ?? [];
    const isFilterActive = personalizationContext?.isFilterActive;

    // Update the theme context to use the authenticated user profile
    const showContent = () => {
      // If personalization is disabled, show all content
      if (!isPersonalized) return true;

      // If no user profile, show all content
      if (!userProfile) return true;

      // Check hardware requirements
      const hasRequiredHardware = forHardware.length === 0 || forHardware.some(hw => {
        // First check if the user has this hardware
        let hasHardware = false;
        switch(hw) {
          case 'mobile':
            hasHardware = userProfile.has_mobile;
            break;
          case 'laptop':
            hasHardware = userProfile.has_laptop;
            break;
          case 'physical_robot':
            hasHardware = userProfile.has_physical_robot;
            break;
          case 'other':
            hasHardware = userProfile.has_other_hardware && userProfile.has_other_hardware.trim() !== '';
            break;
          default:
            hasHardware = false;
        }

        // If user has this hardware, check if it's enabled in active filters
        if (hasHardware && isFilterActive) {
          return isFilterActive(hw) ?? true; // If filter system is available, check if enabled
        }

        return hasHardware; // Otherwise, just check if user has the hardware
      });

      // Check experience requirements
      const hasRequiredExperience = forExperience.length === 0 || forExperience.some(exp => {
        if (!userProfile.web_dev_experience) return false;

        // Define experience hierarchy: beginner < intermediate < experienced < expert
        const experienceLevels = {
          'beginner': 0,
          'intermediate': 1,
          'experienced': 2,
          'expert': 3
        };

        const userLevel = experienceLevels[userProfile.web_dev_experience] || 0;
        const requiredLevel = experienceLevels[exp] || 0;

        return userLevel >= requiredLevel;
      });

      return hasRequiredHardware && hasRequiredExperience;
    };

    // Determine if content should be shown based on user profile
    const contentVisible = showContent();

    // Show content if personalization is enabled and user meets requirements,
    // or show all content if personalization is disabled or contexts are not available
    if (contentVisible) {
      return <div className={`conditional-content ${className}`}>{children}</div>;
    }

    // Show fallback if provided, otherwise show nothing
    return fallback || null;
  } catch (error) {
    // If there are any issues with context or imports, show the content by default
    // This ensures that the component doesn't break the MDX rendering
    console.warn('ConditionalContent: Error accessing context, showing content by default', error);
    return <div className={`conditional-content ${className}`}>{children}</div>;
  }
};

export default ConditionalContent;