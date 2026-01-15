// API Configuration
// In Docusaurus, we can't rely on process.env in browser, so we use a constants file
// The API URL can be configured here and changed based on deployment environment

export const API_BASE_URL = typeof window !== 'undefined'
  ? (window.location.hostname === 'localhost'
      ? 'http://localhost:8000'  // Use absolute URL for direct API access
      : (window.API_CONFIG?.BASE_URL || process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000'))
  : process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';

// For development with proxy, using relative paths works when you set up a proxy
// For production, it will use the absolute URL
// export const API_BASE_URL = 'https://your-production-api.com';