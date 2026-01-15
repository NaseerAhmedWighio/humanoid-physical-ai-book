import React from 'react';
import PersonalizationToggle from './PersonalizationToggle';

const NavigationSidebar = () => {
  return (
    <div className="navigation-sidebar">
      <nav className="sidebar-nav">
        <ul className="nav-list">
          <li className="nav-item">
            <a href="/">Home</a>
          </li>
          <li className="nav-item">
            <a href="/module-1">Module 1</a>
          </li>
          <li className="nav-item">
            <a href="/module-2">Module 2</a>
          </li>
          <li className="nav-item">
            <a href="/exercises">Exercises</a>
          </li>
          <li className="nav-item">
            <a href="/resources">Resources</a>
          </li>
        </ul>
      </nav>

      {/* Personalization Toggle Section */}
      <div className="personalization-section">
        <h3>Personalization</h3>
        <PersonalizationToggle />
      </div>

      <div className="sidebar-footer">
        <p>Humanoid AI Textbook</p>
      </div>
    </div>
  );
};

export default NavigationSidebar;