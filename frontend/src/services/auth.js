// Better Auth client setup
import { createAuthClient } from "@better-auth/client";
import { API_BASE_URL } from '../constants/api';
import { useState, useEffect } from 'react';

// Initialize the better-auth client with custom fetch to handle our endpoint structure
const authClient = createAuthClient({
  baseURL: API_BASE_URL,
});

// Create wrapper functions that map to our backend endpoints
const customSignUp = {
  email: async ({ email, password, callbackURL }) => {
    try {
      const response = await fetch(`${API_BASE_URL}/v1/better-auth/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
          // Include any additional fields that might be expected
        }),
      });

      const result = await response.json();

      if (!response.ok) {
        return {
          error: {
            message: result.detail || 'Registration failed'
          }
        };
      }

      // Store the token for future requests
      if (result.session && result.session.access_token) {
        localStorage.setItem('better_auth_token', result.session.access_token);
      }

      return result;
    } catch (error) {
      console.error('Registration API call failed:', error);
      return {
        error: {
          message: error.message || 'Registration failed'
        }
      };
    }
  }
};

const customSignIn = {
  email: async ({ email, password, callbackURL }) => {
    try {
      const response = await fetch(`${API_BASE_URL}/v1/better-auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password
        }),
      });

      const result = await response.json();

      if (!response.ok) {
        return {
          error: {
            message: result.detail || 'Login failed'
          }
        };
      }

      // Store the token for future requests
      if (result.session && result.session.access_token) {
        localStorage.setItem('better_auth_token', result.session.access_token);
      }

      return result;
    } catch (error) {
      console.error('Login API call failed:', error);
      return {
        error: {
          message: error.message || 'Login failed'
        }
      };
    }
  }
};

const customSignOut = async () => {
  try {
    const response = await fetch(`${API_BASE_URL}/v1/better-auth/sign-out`, {
      method: 'POST',
    });

    if (!response.ok) {
      console.error('Sign out failed:', await response.text());
    }

    // Clear the stored token
    localStorage.removeItem('better_auth_token');
    sessionStorage.removeItem('better_auth_token');

    // Return a promise to ensure proper async handling
    return Promise.resolve(true);
  } catch (error) {
    console.error('Sign out API call failed:', error);
    return Promise.resolve(false);
  }
};

// Export the custom methods instead of the default ones
export const signIn = customSignIn;
export const signUp = customSignUp;
export const signOut = customSignOut;

// Create a custom useSession hook since the built-in one might not be available in beta
export const useSession = () => {
  const [session, setSession] = useState({ data: null, isLoading: true, isError: false });

  const fetchSession = async () => {
    try {
      // Try to get session from our custom endpoint
      // The session endpoint likely requires authentication
      // We'll need to send the token if it's stored locally
      const token = localStorage.getItem('better_auth_token') || sessionStorage.getItem('better_auth_token');

      const response = await fetch(`${API_BASE_URL}/v1/better-auth/session`, {
        headers: {
          'Authorization': token ? `Bearer ${token}` : '',
        }
      });

      // Check if the response is 403 (unauthorized) and handle accordingly
      if (response.status === 403) {
        // If 403, user is not authenticated
        setSession({ data: { user: null, session: null }, isLoading: false, isError: false });
      } else if (response.ok) {
        const currentSession = await response.json();
        setSession({ data: currentSession, isLoading: false, isError: false });
      } else {
        setSession({ data: null, isLoading: false, isError: false });
      }
    } catch (error) {
      console.error('Error fetching session:', error);
      setSession({ data: null, isLoading: false, isError: true });
    }
  };

  useEffect(() => {
    // Fetch session on mount
    fetchSession();
  }, []);

  useEffect(() => {
    // Listen for storage changes to update session when token changes
    const handleStorageChange = (e) => {
      if (e.key === 'better_auth_token') {
        fetchSession();
      }
    };

    window.addEventListener('storage', handleStorageChange);

    return () => {
      window.removeEventListener('storage', handleStorageChange);
    };
  }, []);

  return session;
};

// Also export the client itself in case it's needed
export { authClient };