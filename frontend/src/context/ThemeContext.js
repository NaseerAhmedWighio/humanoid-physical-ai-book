import React, { createContext, useContext, useEffect, useState } from 'react';
import { useBetterAuth } from './BetterAuthContext';

const ThemeContext = createContext();

export const useTheme = () => {
  const context = useContext(ThemeContext);
  if (!context) {
    throw new Error('useTheme must be used within a ThemeProvider');
  }
  return context;
};

export const ThemeProvider = ({ children }) => {
  const { user, loading } = useBetterAuth();
  const [userProfile, setUserProfile] = useState(null);

  useEffect(() => {
    if (user) {
      // Set user profile from BetterAuth context
      setUserProfile({
        has_mobile: user.has_mobile || false,
        has_laptop: user.has_laptop || false,
        has_physical_robot: user.has_physical_robot || false,
        has_other_hardware: user.has_other_hardware || null,
        web_dev_experience: user.web_dev_experience || null,
        name: user.name || user.email?.split('@')[0] || null,
        email: user.email || null,
        profile_picture_url: user.image || null
      });
    } else if (!loading) {
      // Set default profile for non-authenticated users
      setUserProfile({
        has_mobile: false,
        has_laptop: false,
        has_physical_robot: false,
        has_other_hardware: null,
        web_dev_experience: null,
        name: null,
        email: null,
        profile_picture_url: null
      });
    }
  }, [user, loading]);

  // Function to check if content should be shown based on user profile
  const shouldShowContent = (forHardware = [], forExperience = []) => {
    if (!userProfile) return true; // Show to all if no profile data

    // Check hardware requirements
    const hasRequiredHardware = forHardware.length === 0 || forHardware.some(hw => {
      switch(hw) {
        case 'mobile':
          return userProfile.has_mobile;
        case 'laptop':
          return userProfile.has_laptop;
        case 'physical_robot':
          return userProfile.has_physical_robot;
        case 'other':
          return userProfile.has_other_hardware && userProfile.has_other_hardware.trim() !== '';
        default:
          return false;
      }
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

  const value = {
    userProfile,
    shouldShowContent
  };

  return (
    <ThemeContext.Provider value={value}>
      {children}
    </ThemeContext.Provider>
  );
};