import React, { useEffect } from 'react';
import { useBetterAuth } from '../../context/BetterAuthContext';
import { useHistory, useLocation } from '@docusaurus/router';

// Component to protect routes that should only be accessible when NOT authenticated
const ProtectedRoute = ({ children, redirectTo = '/' }) => {
  const { user, loading } = useBetterAuth();
  const history = useHistory();
  const location = useLocation();

  useEffect(() => {
    if (!loading && user) {
      // If user is already logged in, redirect them
      history.replace(redirectTo);
    }
  }, [user, loading, history, redirectTo]);

  // Show loading state while checking auth status
  if (loading) {
    return (
      <div className="auth-container">
        <div className="auth-form">
          <div className="auth-form__loading">
            <div className="auth-form__spinner"></div>
            <p>Checking authentication status...</p>
          </div>
        </div>
      </div>
    );
  }

  // If user is not logged in, show the children
  if (!user) {
    return children;
  }

  // If user is logged in, return null while redirect happens
  return null;
};

export default ProtectedRoute;