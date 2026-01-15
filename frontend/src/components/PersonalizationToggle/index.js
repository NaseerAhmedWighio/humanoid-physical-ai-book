import React, { useState, useEffect, useCallback, useMemo } from 'react';
import { usePersonalization } from '../../context/PersonalizationContext';
import { useBetterAuth } from '../../context/BetterAuthContext';
import { useLocation } from '@docusaurus/router';
import './PersonalizationToggle.css';

const PersonalizationToggle = () => {
  const { isPersonalized, togglePersonalization, getPersonalizedContent, userPreferences } = usePersonalization();
  const { user } = useBetterAuth();
  const location = useLocation();
  const [expanded, setExpanded] = useState(false);

  // Check if we should show the toggle on this page
  // Don't show on homepage, auth pages, or intro documentation
  const isHomePage = location.pathname === '/' || location.pathname === '/index.html';
  const isAuthPage = location.pathname.includes('/signin') ||
                    location.pathname.includes('/signup') ||
                    location.pathname.includes('/auth/');
  const isDocsIntroPage = location.pathname.startsWith('/docs/intro');
  const is404Page = location.pathname.includes('/404');

  // Show personalized content options based on user profile
  const generatePersonalizedOptions = useCallback(() => {
    if (!user) return [];

    const options = [];

    if (user?.has_physical_robot) {
      options.push({
        id: 'physical_robot',
        title: 'Physical Robot Content',
        description: 'Content specifically designed for users with physical robots',
        content: [
          'Real-world testing examples',
          'Hardware-specific debugging techniques',
          'Physical robot calibration procedures'
        ]
      });
    }

    if (user?.has_laptop) {
      options.push({
        id: 'simulation',
        title: 'Simulation Content',
        description: 'Content focused on simulation environments',
        content: [
          'Gazebo simulation tutorials',
          'ROS 2 launch files for simulation',
          'Virtual testing environments'
        ]
      });
    }

    if (user?.has_mobile) {
      options.push({
        id: 'mobile',
        title: 'Mobile Development Content',
        description: 'Content for mobile device development',
        content: [
          'Mobile ROS applications',
          'Remote robot control',
          'Mobile interfaces for robotics'
        ]
      });
    }

    if (user?.web_dev_experience) {
      options.push({
        id: 'experience',
        title: `Experience Level: ${user.web_dev_experience}`,
        description: `Content tailored to ${user.web_dev_experience} level developers`,
        content: [
          'Appropriate complexity examples',
          'Relevant best practices',
          'Level-appropriate challenges'
        ]
      });
    }

    if (user?.has_other_hardware) {
      options.push({
        id: 'other_hardware',
        title: 'Other Hardware Content',
        description: `Content for users with: ${user.has_other_hardware}`,
        content: [
          'Hardware-specific tutorials',
          'Integration guides',
          'Custom development tips'
        ]
      });
    }

    return options;
  }, [user]);

  const personalizedOptions = useMemo(() => generatePersonalizedOptions(), [generatePersonalizedOptions]);

  // Don't render if this is an excluded page
  if (isHomePage || isAuthPage || isDocsIntroPage || is404Page) {
    return null;
  }

  return (
    <div className="personalization-toggle-container">
      <div className="personalization-toggle" role="region" aria-label="Personalization Options">
        <div className="personalization-toggle-header">
          <span className="toggle-label">
            {isPersonalized ? 'Personalized Content' : 'Original Content'}
          </span>
          <label className="switch">
            <input
              type="checkbox"
              checked={!!isPersonalized}
              onChange={togglePersonalization}
              aria-label={isPersonalized ? 'Personalization is ON' : 'Personalization is OFF'}
            />
            <span className="slider round"></span>
          </label>
          <span className="toggle-status">
            {isPersonalized ? 'ON' : 'OFF'}
          </span>
        </div>

        {/* Expandable section showing user's personalization options */}
        {user && personalizedOptions.length > 0 && (
          <div className="personalization-options">
            <button
              className="expand-toggle"
              onClick={() => setExpanded(prev => !prev)}
              aria-expanded={expanded}
              aria-controls="personalization-options-list"
            >
              {expanded ? 'Hide Options' : 'Show Personalized Options'}
              <span className="expand-icon">{expanded ? '▲' : '▼'}</span>
            </button>

            {expanded && (
              <div className="options-list" id="personalization-options-list">
                {personalizedOptions.map((option) => (
                  <div key={option.id} className="option-item">
                    <h4>{option.title}</h4>
                    <p>{option.description}</p>
                    <ul>
                      {option.content.map((item, idx) => (
                        <li key={`${option.id}-${idx}`}>{item}</li>
                      ))}
                    </ul>
                  </div>
                ))}
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export default PersonalizationToggle;