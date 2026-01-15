import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import { API_BASE_URL } from '../constants/api';
import './SearchModal.css'; // Use the same CSS as SearchModal for consistency
import Notification from './Notification';

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

const CustomSearch = ({
  placeholder = "Search the textbook content...",
  onSearchComplete,
  onResultClick,
  showFilters = true,
  autoFocus = false
}) => {
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedCategory, setSelectedCategory] = useState('all');
  const [searchResults, setSearchResults] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [showNotification, setShowNotification] = useState(false);
  const [notificationMessage, setNotificationMessage] = useState('');
  const [notificationType, setNotificationType] = useState('info');
  const inputRef = useRef(null);

  // Search content using backend API
  const searchContent = async (query) => {
    if (!query.trim()) {
      setSearchResults([]);
      setError(null);
      if (onSearchComplete) onSearchComplete([]);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Use the same API endpoint as SearchModal.js for consistency
      const response = await axios.post(`${API_BASE_URL}/content/search`, {
        query: query,
        limit: 20
      });

      if (response.data.error) {
        // Show notification about service unavailability
        setNotificationMessage('Search service is temporarily unavailable. Using local search.');
        setNotificationType('warning');
        setShowNotification(true);

        // Fallback to local search
        const fallbackResults = getFilteredResults(query);
        setSearchResults(fallbackResults);
        if (onSearchComplete) onSearchComplete(fallbackResults);
      } else {
        // Format the search results from the API
        const formattedResults = response.data.results.map((result, index) => {
          // Extract path and title from the content metadata
          let path = '/docs/intro'; // default fallback
          let title = result.snippet || `Search result ${index + 1}`;

          // Try to create a meaningful title from the snippet
          if (result.title) {
            title = result.title;
          }

          return {
            title: title,
            path: path,
            content: result.snippet,
            score: result.relevance_score || 0
          };
        });

        setSearchResults(formattedResults);
        if (onSearchComplete) onSearchComplete(formattedResults);
      }
    } catch (error) {
      // Use the new error handling function
      handleSearchError(error);

      // Fallback to local search if API fails
      try {
        const fallbackResults = getFilteredResults(query);
        setSearchResults(fallbackResults);
        if (onSearchComplete) onSearchComplete(fallbackResults);

        // Show notification about using fallback
        setNotificationMessage('Search service unavailable. Using local search.');
        setNotificationType('warning');
        setShowNotification(true);
      } catch (fallbackError) {
        console.error('Fallback search error:', fallbackError);
        setSearchResults([]);
        if (onSearchComplete) onSearchComplete([]);

        // Show error notification
        setNotificationMessage('Search is temporarily unavailable.');
        setNotificationType('error');
        setShowNotification(true);
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

  // Handle search API unavailability with proper error handling
  const handleSearchError = (error) => {
    console.error('Search API error:', error);

    // Determine the type of error and provide appropriate message
    let errorMessage = 'Search is temporarily unavailable. Please try again later.';

    if (error.response) {
      // Server responded with error status
      if (error.response.status === 404) {
        errorMessage = 'Search service is currently unavailable.';
      } else if (error.response.status === 500) {
        errorMessage = 'Search service is experiencing issues. Please try again later.';
      } else if (error.response.status === 429) {
        errorMessage = 'Too many search requests. Please wait before trying again.';
      }
    } else if (error.request) {
      // Request was made but no response received
      errorMessage = 'Unable to connect to search service. Please check your connection.';
    } else {
      // Other error
      errorMessage = 'An error occurred while searching. Please try again.';
    }

    setError(errorMessage);
    return errorMessage;
  };

  // Handle search query changes
  useEffect(() => {
    const delayDebounce = setTimeout(() => {
      if (searchQuery.trim()) {
        searchContent(searchQuery);
      } else {
        setSearchResults([]);
        setError(null);
        if (onSearchComplete) onSearchComplete([]);
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
      if (onSearchComplete) onSearchComplete(results);
    }
  }, [selectedCategory, searchQuery]);

  // Auto-focus input if requested
  useEffect(() => {
    if (autoFocus && inputRef.current) {
      inputRef.current.focus();
    }
  }, [autoFocus]);

  const handleResultClick = (result) => {
    if (onResultClick) {
      onResultClick(result);
    }
  };

  const handleNotificationClose = () => {
    setShowNotification(false);
  };

  // Use search results if available, otherwise use filtered results
  const displayResults = searchResults.length > 0 ? searchResults : getFilteredResults();

  return (
    <div className="custom-search-container">
      {showNotification && (
        <Notification
          message={notificationMessage}
          type={notificationType}
          onClose={handleNotificationClose}
        />
      )}
      <div className="search-input-container">
        <input
          ref={inputRef}
          type="text"
          className="search-input"
          placeholder={placeholder}
          value={searchQuery}
          onChange={(e) => setSearchQuery(e.target.value)}
        />
        {isLoading && (
          <div className="search-loading-indicator">
            <div className="loading-spinner-small"></div>
          </div>
        )}
      </div>

      {showFilters && (
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
      )}

      <div className="search-results">
        {error && (
          <div className="search-error">
            {error}
          </div>
        )}
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
                onClick={(e) => {
                  e.preventDefault();
                  handleResultClick(item);
                  // In a real implementation, you might want to navigate differently
                  window.location.href = item.path;
                }}
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
  );
};

export default CustomSearch;