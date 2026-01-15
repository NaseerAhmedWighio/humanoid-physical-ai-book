import React, { createContext, useContext, useEffect, useState } from 'react';
import { useSession, signIn, signUp, signOut } from '../services/auth';
import { API_BASE_URL } from '../constants/api';

const BetterAuthContext = createContext();

export const useBetterAuth = () => {
  const context = useContext(BetterAuthContext);
  if (!context) {
    throw new Error('useBetterAuth must be used within a BetterAuthProvider');
  }
  return context;
};

export const BetterAuthProvider = ({ children }) => {
  const { data: session, isLoading, isError } = useSession();
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  // Sync the BetterAuth session with our local state
  useEffect(() => {
    if (isLoading) {
      setLoading(true);
      return;
    }

    if (session && session.data && session.data.user) {
      // Map BetterAuth user data to our expected format with defensive checks
      setUser({
        id: session.data.user.id || null,
        email: session.data.user.email || null,
        name: session.data.user.name || (session.data.user.email && session.data.user.email.split('@')[0]) || 'User',
        image: session.data.user.image || null,
        // Map custom fields if they exist in session data
        has_mobile: session.data.user.has_mobile || false,
        has_laptop: session.data.user.has_laptop || false,
        has_physical_robot: session.data.user.has_physical_robot || false,
        has_other_hardware: session.data.user.has_other_hardware || '',
        web_dev_experience: session.data.user.web_dev_experience || ''
      });
    } else {
      setUser(null);
    }

    setLoading(false);
  }, [session, isLoading]);

  // BetterAuth login function
  const login = async (email, password) => {
    try {
      console.log('Logging in user with email:', email); // Debug log

      // Add defensive check for signIn and signIn.email
      if (!signIn || typeof signIn.email !== 'function') {
        console.error('signIn.email function is not available');
        return {
          success: false,
          error: 'Login service is not available'
        };
      }

      // Validate input parameters
      if (!email || !password) {
        console.error('Email and password are required');
        return {
          success: false,
          error: 'Email and password are required'
        };
      }

      const result = await signIn.email({
        email,
        password,
        callbackURL: '/' // Redirect to home after login
      });

      console.log('Login result:', result); // Debug log

      // Add defensive check to handle undefined results
      if (!result) {
        return {
          success: false,
          error: 'Login failed - no response received'
        };
      }

      if (result?.error) {
        return {
          success: false,
          error: result.error?.message || result.error || 'Login failed'
        };
      }

      // Handle different response structures for login
      let userData = null;
      if (result?.user) {
        userData = result.user;
      } else if (result?.data?.user) {
        userData = result.data.user;
      } else if (result && typeof result === 'object' && !result.error) {
        // If result is an object without error, it might be the user data itself
        userData = result;
      }

      // Update user state directly for immediate UI update
      if (userData) {
        setUser({
          id: userData.id || null,
          email: userData.email || null,
          name: userData.name || (userData.email && userData.email.split('@')[0]) || 'User',
          image: userData.image || null,
          has_mobile: userData.has_mobile || false,
          has_laptop: userData.has_laptop || false,
          has_physical_robot: userData.has_physical_robot || false,
          has_other_hardware: userData.has_other_hardware || '',
          web_dev_experience: userData.web_dev_experience || ''
        });
      }

      // Manually trigger a storage event to update the session hook
      const token = localStorage.getItem('better_auth_token') || sessionStorage.getItem('better_auth_token');
      if (token) {
        window.dispatchEvent(new StorageEvent('storage', {
          key: 'better_auth_token',
          oldValue: null, // not important for our use case
          newValue: token,
          url: window.location.href
        }));
      }

      return {
        success: true,
        user: userData || null
      };
    } catch (error) {
      console.error('Login failed:', error);
      return {
        success: false,
        error: error.message || 'Login failed'
      };
    }
  };

  // BetterAuth register function
  const register = async (email, password, preferences = {}) => {
    try {
      console.log('Registering user with email:', email); // Debug log

      // Check if we want to use the backend API directly for combined registration
      // This is used when we need to register with preferences in one request
      if (preferences && Object.keys(preferences).length > 0) {
        // Use the better-auth API to register with preferences
        const response = await fetch(`${API_BASE_URL}/v1/better-auth/register`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            email,
            password,
            ...preferences
          }),
        });

        // Check if the response is JSON before parsing
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
          throw new Error('Server returned non-JSON response');
        }

        const result = await response.json();

        if (response.ok) {
          // Store the token if returned
          if (result.access_token) {
            localStorage.setItem('better_auth_token', result.access_token);
            // Trigger storage event to update session
            window.dispatchEvent(new StorageEvent('storage', {
              key: 'better_auth_token',
              oldValue: null,
              newValue: result.access_token,
              url: window.location.href
            }));
          }

          // Update user state with the returned user data
          const userData = {
            id: result.id || null,
            email: result.email || null,
            name: result.email && result.email.split('@')[0] || 'User',
            has_mobile: result.has_mobile || false,
            has_laptop: result.has_laptop || false,
            has_physical_robot: result.has_physical_robot || false,
            has_other_hardware: result.has_other_hardware || '',
            web_dev_experience: result.web_dev_experience || ''
          };

          setUser(userData);

          return {
            success: true,
            user: userData
          };
        } else {
          return {
            success: false,
            error: result.detail || 'Registration failed'
          };
        }
      } else {
        // Use the original better-auth registration flow for basic registration
        // Add defensive check for signUp and signUp.email
        if (!signUp || typeof signUp.email !== 'function') {
          console.error('signUp.email function is not available');
          return {
            success: false,
            error: 'Registration service is not available'
          };
        }

        // Validate input parameters
        if (!email || !password) {
          console.error('Email and password are required');
          return {
            success: false,
            error: 'Email and password are required'
          };
        }

        const result = await signUp.email({
          email,
          password,
          callbackURL: '/purpose-selection' // Redirect after registration
        });

        console.log('Registration result:', result); // Debug log

        // Add defensive check to handle undefined results
        if (!result) {
          return {
            success: false,
            error: 'Registration failed - no response received'
          };
        }

        if (result?.error) {
          return {
            success: false,
            error: result.error?.message || result.error || 'Registration failed'
          };
        }

        // The response structure from better-auth may vary, let's handle different possibilities
        let userData = null;
        if (result?.user) {
          userData = result.user;
        } else if (result?.data?.user) {
          userData = result.data.user;
        } else if (result && typeof result === 'object' && !result.error) {
          // If result is an object without error, it might be the user data itself
          userData = result;
        }

        // Update user state directly for immediate UI update
        if (userData) {
          setUser({
            id: userData.id || null,
            email: userData.email || null,
            name: userData.name || (userData.email && userData.email.split('@')[0]) || 'User',
            image: userData.image || null,
            has_mobile: userData.has_mobile || false,
            has_laptop: userData.has_laptop || false,
            has_physical_robot: userData.has_physical_robot || false,
            has_other_hardware: userData.has_other_hardware || '',
            web_dev_experience: userData.web_dev_experience || ''
          });
        }

        // Manually trigger a storage event to update the session hook
        const token = localStorage.getItem('better_auth_token') || sessionStorage.getItem('better_auth_token');
        if (token) {
          window.dispatchEvent(new StorageEvent('storage', {
            key: 'better_auth_token',
            oldValue: null, // not important for our use case
            newValue: token,
            url: window.location.href
          }));
        }

        return {
          success: true,
          user: userData || null
        };
      }
    } catch (error) {
      console.error('Registration failed:', error);
      return {
        success: false,
        error: error.message || 'Registration failed'
      };
    }
  };

  // BetterAuth logout function
  const logout = async () => {
    try {
      await signOut();
      // Clear stored token - this will trigger the storage event listener in other tabs
      localStorage.removeItem('better_auth_token');
      sessionStorage.removeItem('better_auth_token');
      // Also manually trigger a storage event for the current tab to update session
      window.dispatchEvent(new StorageEvent('storage', {
        key: 'better_auth_token',
        oldValue: 'token', // not important for our use case
        newValue: null,
        url: window.location.href
      }));
      // Reset user state
      setUser(null);
    } catch (error) {
      console.error('Logout failed:', error);
    }
  };

  const updateUserPreferences = async (preferences) => {
    try {
      // Update preferences via API call
      const token = localStorage.getItem('better_auth_token') || sessionStorage.getItem('better_auth_token');

      if (!token) {
        return {
          success: false,
          error: 'No authentication token found'
        };
      }

      const response = await fetch(`${API_BASE_URL}/v1/better-auth/update-preferences`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify(preferences)
      });

      const contentType = response.headers.get('content-type');
      if (!contentType || !contentType.includes('application/json')) {
        throw new Error('Server returned non-JSON response');
      }

      const result = await response.json();

      if (response.ok) {
        // Update local user state with new preferences
        const updatedUser = {
          ...user,
          ...preferences
        };
        setUser(updatedUser);

        return {
          success: true,
          user: updatedUser
        };
      } else {
        return {
          success: false,
          error: result.detail || 'Failed to update preferences'
        };
      }
    } catch (error) {
      console.error('Failed to update preferences:', error);
      return {
        success: false,
        error: error.message || 'Failed to update preferences'
      };
    }
  };

  const value = {
    user,
    login,
    register,
    logout,
    updateUserPreferences,
    loading: loading || isLoading
  };

  return (
    <BetterAuthContext.Provider value={value}>
      {children}
    </BetterAuthContext.Provider>
  );
};