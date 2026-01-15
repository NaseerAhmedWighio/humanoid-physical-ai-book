import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import axios from 'axios';
import { API_BASE_URL } from '../constants/api';
import './SearchModal.css';

const SearchModal = ({ isOpen, onClose }) => {
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedCategory, setSelectedCategory] = useState('all');
  const [searchResults, setSearchResults] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const inputRef = useRef(null);
  const location = useLocation();

  // Search data - topics from the website (fallback for non-content search)
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

  // Search content using backend API
  const searchContent = async (query) => {
    if (!query.trim()) {
      // If no query, show category-based results
      setSearchResults([]);
      return;
    }

    setIsLoading(true);
    try {
      const response = await axios.post(`${API_BASE_URL}/content/search`, {
        query: query,
        limit: 20
      });

      // Format the search results from the API
      const formattedResults = response.data.results.map((result, index) => {
        // Extract path and title from the content metadata
        let path = '/docs/intro'; // default fallback
        let title = result.content || `Search result ${index + 1}`;

        // Try to extract path from metadata
        if (result.metadata && result.metadata.url) {
          path = result.metadata.url;
        } else if (result.metadata && result.metadata.path) {
          path = result.metadata.path;
        }

        // Create a title from the content if not available
        if (!result.metadata?.title && result.content) {
          title = result.content.substring(0, 100) + (result.content.length > 100 ? '...' : '');
        } else if (result.metadata?.title) {
          title = result.metadata.title;
        }

        return {
          title: title,
          path: path,
          content: result.content,
          score: result.score || 0
        };
      });

      setSearchResults(formattedResults);
    } catch (error) {
      console.error('Search error:', error);
      // Fallback to local search if API fails
      try {
        const fallbackResults = getFilteredResults(query);
        setSearchResults(fallbackResults);
      } catch (fallbackError) {
        console.error('Fallback search error:', fallbackError);
        setSearchResults([]);
      }
    } finally {
      setIsLoading(false);
    }
  };

  // Filter search topics based on query and category (fallback)
  const getFilteredResults = (query = searchQuery) => {
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

    if (query.trim()) {
      results = results.filter(item =>
        item.title.toLowerCase().includes(query.toLowerCase())
      );
    }

    return results;
  };

  // Handle search query changes
  useEffect(() => {
    const delayDebounce = setTimeout(() => {
      if (searchQuery.trim()) {
        searchContent(searchQuery);
      } else {
        setSearchResults([]);
      }
    }, 300); // 300ms delay to avoid excessive API calls

    return () => clearTimeout(delayDebounce);
  }, [searchQuery]);

  // Handle category changes
  useEffect(() => {
    if (!searchQuery.trim()) {
      // Only apply category filtering when there's no search query
      const results = getFilteredResults();
      setSearchResults(results);
    }
  }, [selectedCategory, searchQuery]);

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

  // Use search results if available, otherwise use filtered results
  const displayResults = searchResults.length > 0 ? searchResults : getFilteredResults();

  return (
    <div className="search-modal-overlay" onClick={onClose}>
      <div className="search-modal" onClick={(e) => e.stopPropagation()}>
        <div className="search-header">
          <div className="search-input-container">
            <input
              ref={inputRef}
              type="text"
              className="search-input"
              placeholder="Search the textbook content..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              onClick={(e) => e.stopPropagation()}
            />
            <div className="search-shortcut">
              <kbd>Ctrl</kbd>
              <span>+</span>
              <kbd>K</kbd>
            </div>
              <button className="search-close" onClick={onClose}>
            ×
          </button>
          </div>
        
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
          {isLoading ? (
            <div className="search-loading">
              <div className="loading-spinner"></div>
              <div>Searching content...</div>
            </div>
          ) : displayResults.length > 0 ? (
            displayResults.map((item, index) => {
              // Create a stable key based on content to avoid DOM reconciliation issues
              const resultKey = item.path + (item.title || '').substring(0, 20) + index;
              return (
                <a
                  key={resultKey}
                  href={item.path}
                  className="search-result-item"
                  onClick={onClose}
                >
                  <div className="search-result-title">{item.title}</div>
                  <div className="search-result-path">{item.path}</div>
                  {item.content && (
                    <div className="search-result-preview">
                      {item.content.substring(0, 150)}...
                    </div>
                  )}
                </a>
              );
            })
          ) : searchQuery.trim() ? (
            <div className="search-no-results">
              No results found for "{searchQuery}"
            </div>
          ) : (
            <div className="search-no-results">
              Start typing to search the textbook content
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default SearchModal;