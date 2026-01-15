import React from 'react';
import { useTheme } from '../../context/ThemeContext';
import './ConditionalContent.css';

/**
 * ConditionalContent component renders different content based on user's profile
 * It shows content based on hardware availability and web development experience
 */
const ConditionalContent = ({ children, forHardware = [], forExperience = [], fallback = null, className = '' }) => {
  const { shouldShowContent } = useTheme();

  // Determine if content should be shown based on user profile
  const showContent = shouldShowContent(forHardware, forExperience);

  // Show content if user meets requirements
  if (showContent) {
    return <div className={`conditional-content ${className}`}>{children}</div>;
  }

  // Show fallback if provided, otherwise show nothing
  return fallback || null;
};

export default ConditionalContent;