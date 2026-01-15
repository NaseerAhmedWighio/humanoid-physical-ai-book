import React, { createContext, useContext, useState, useEffect } from 'react';
import { useBetterAuth } from './BetterAuthContext';

const PersonalizationContext = createContext();

export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (!context) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};

export const PersonalizationProvider = ({ children }) => {
  const { user } = useBetterAuth();
  const [isPersonalized, setIsPersonalized] = useState(true);
  const [activeFilters, setActiveFilters] = useState({});
  const [personalizedContent, setPersonalizedContent] = useState({});

  // Generate personalized content based on user preferences
  useEffect(() => {
    if (user) {
      const content = generatePersonalizedContent(user);
      setPersonalizedContent(content);

      // Set default active filters based on user's hardware
      const defaultFilters = {};
      if (user.has_physical_robot) defaultFilters.physical_robot = true;
      if (user.has_laptop) defaultFilters.simulation = true;
      if (user.has_mobile) defaultFilters.mobile = true;
      if (user.has_other_hardware) defaultFilters.other_hardware = true;

      setActiveFilters(defaultFilters);
    }
  }, [user]);

  const generatePersonalizedContent = (userPreferences) => {
    const content = {};

    // Personalize content based on hardware availability
    if (userPreferences.has_physical_robot) {
      content.hardware_specific = {
        title: "Physical Robot Content",
        description: "Content specifically designed for users with physical robots",
        examples: [
          "Real-world testing examples",
          "Hardware-specific debugging techniques",
          "Physical robot calibration procedures"
        ]
      };
    } else if (userPreferences.has_laptop) {
      content.hardware_specific = {
        title: "Simulation Content",
        description: "Content focused on simulation environments",
        examples: [
          "Gazebo simulation tutorials",
          "ROS 2 launch files for simulation",
          "Virtual testing environments"
        ]
      };
    } else if (userPreferences.has_mobile) {
      content.hardware_specific = {
        title: "Mobile Development Content",
        description: "Content for mobile device development",
        examples: [
          "Mobile ROS applications",
          "Remote robot control",
          "Mobile interfaces for robotics"
        ]
      };
    }

    // Personalize based on experience level
    if (userPreferences.web_dev_experience) {
      content.experience_level = userPreferences.web_dev_experience;
    }

    // Personalize content based on other hardware
    if (userPreferences.has_other_hardware) {
      content.other_hardware = userPreferences.has_other_hardware;
    }

    return content;
  };

  const getPersonalizedContent = (topic) => {
    if (!isPersonalized || !user) return null;
    return personalizedContent[topic] || null;
  };

  const togglePersonalization = () => {
    setIsPersonalized(!isPersonalized);
  };

  // New methods for content filtering
  const toggleFilter = (filterId) => {
    setActiveFilters(prev => ({
      ...prev,
      [filterId]: !prev[filterId]
    }));
  };

  const isFilterActive = (filterId) => {
    return activeFilters[filterId] || false;
  };

  const getActiveFilters = () => {
    return Object.keys(activeFilters).filter(key => activeFilters[key]);
  };

  const value = {
    isPersonalized,
    togglePersonalization,
    getPersonalizedContent,
    userPreferences: user,
    personalizedContent,
    // New filtering methods
    activeFilters,
    toggleFilter,
    isFilterActive,
    getActiveFilters
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
};