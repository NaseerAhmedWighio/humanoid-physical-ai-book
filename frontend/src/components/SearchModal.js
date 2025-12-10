import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import './SearchModal.css';

const SearchModal = ({ isOpen, onClose }) => {
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedCategory, setSelectedCategory] = useState('all');
  const inputRef = useRef(null);
  const location = useLocation();

  // Search data - topics from the website
  const searchTopics = {
    modules: [
      { title: 'Module 1: Introduction to ROS 2', path: '/docs/module-1-ros2' },
      { title: 'Module 2: Simulation with Gazebo and Unity', path: '/docs/module-2-simulation' },
      { title: 'Module 3: NVIDIA Isaac for Humanoid Control', path: '/docs/module-3-nvidia-isaac' },
      { title: 'Module 4: Vision Language Action (VLA) Models', path: '/docs/module-4-vla' },
    ],
    weeks: [
      { title: 'Week 1-3: ROS 2 Fundamentals', path: '/docs/weeks/week-01-02' },
      { title: 'Week 4-6: Simulation Environments', path: '/docs/weeks/week-03' },
      { title: 'Week 7-9: NVIDIA Isaac Integration', path: '/docs/weeks/week-04-05' },
      { title: 'Week 10-13: AI Integration', path: '/docs/weeks/week-10-13' },
    ],
    appendices: [
      { title: 'Appendix A: Mathematical Foundations', path: '/docs/appendices/mathematical-foundations' },
      { title: 'Appendix B: Hardware Specifications', path: '/docs/appendices/hardware-specifications' },
      { title: 'Appendix C: Software Installation Guide', path: '/docs/appendices/software-installation' },
      { title: 'Appendix D: Troubleshooting Guide', path: '/docs/appendices/troubleshooting' },
      { title: 'Appendix G: Safety Guidelines', path: '/docs/appendices/safety-guidelines' },
      { title: 'Appendix H: Ethics in Robotics', path: '/docs/appendices/ethics' },
      { title: 'Appendix J: Additional Resources', path: '/docs/appendices/additional-resources' },
    ],
    concepts: [
      { title: 'ROS 2 Architecture', path: '/docs/module-1-ros2' },
      { title: 'Humanoid Robot Design', path: '/docs/intro' },
      { title: 'Simulation Best Practices', path: '/docs/appendices/simulation-best-practices' },
      { title: 'NVIDIA Isaac Integration', path: '/docs/module-3-nvidia-isaac' },
      { title: 'Vision Language Action Models', path: '/docs/module-4-vla' },
      { title: 'Human-Robot Interaction', path: '/docs/intro' },
    ]
  };

  // Filter search results based on query and category
  const getFilteredResults = () => {
    let results = [];

    if (selectedCategory === 'all' || selectedCategory === 'modules') {
      results = [...results, ...searchTopics.modules];
    }
    if (selectedCategory === 'all' || selectedCategory === 'weeks') {
      results = [...results, ...searchTopics.weeks];
    }
    if (selectedCategory === 'all' || selectedCategory === 'appendices') {
      results = [...results, ...searchTopics.appendices];
    }
    if (selectedCategory === 'all' || selectedCategory === 'concepts') {
      results = [...results, ...searchTopics.concepts];
    }

    if (searchQuery.trim()) {
      results = results.filter(item =>
        item.title.toLowerCase().includes(searchQuery.toLowerCase())
      );
    }

    return results;
  };

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.ctrlKey && e.key === 'k') {
        e.preventDefault();
        if (isOpen) {
          onClose();
        } else {
          // This would be handled by the navbar component
        }
      }
      if (e.key === 'Escape' && isOpen) {
        onClose();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);

  if (!isOpen) return null;

  const filteredResults = getFilteredResults();

  return (
    <div className="search-modal-overlay" onClick={onClose}>
      <div className="search-modal" onClick={(e) => e.stopPropagation()}>
        <div className="search-header">
          <input
            ref={inputRef}
            type="text"
            className="search-input"
            placeholder="Search topics, modules, weeks..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
            onClick={(e) => e.stopPropagation()}
          />
          <button className="search-close" onClick={onClose}>
            ×
          </button>
        </div>

        <div className="search-filters">
          <button
            className={`search-filter ${selectedCategory === 'all' ? 'active' : ''}`}
            onClick={() => setSelectedCategory('all')}
          >
            All
          </button>
          <button
            className={`search-filter ${selectedCategory === 'modules' ? 'active' : ''}`}
            onClick={() => setSelectedCategory('modules')}
          >
            Modules
          </button>
          <button
            className={`search-filter ${selectedCategory === 'weeks' ? 'active' : ''}`}
            onClick={() => setSelectedCategory('weeks')}
          >
            Weeks
          </button>
          <button
            className={`search-filter ${selectedCategory === 'appendices' ? 'active' : ''}`}
            onClick={() => setSelectedCategory('appendices')}
          >
            Appendices
          </button>
          <button
            className={`search-filter ${selectedCategory === 'concepts' ? 'active' : ''}`}
            onClick={() => setSelectedCategory('concepts')}
          >
            Concepts
          </button>
        </div>

        <div className="search-results">
          {filteredResults.length > 0 ? (
            filteredResults.map((item, index) => (
              <a
                key={index}
                href={item.path}
                className="search-result-item"
                onClick={onClose}
              >
                <div className="search-result-title">{item.title}</div>
                <div className="search-result-path">{item.path}</div>
              </a>
            ))
          ) : (
            <div className="search-no-results">
              No results found for "{searchQuery}"
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default SearchModal;