import React, { useEffect } from 'react';
import { useBetterAuth } from '../../context/BetterAuthContext';
import { useHistory } from '@docusaurus/router';

// Component to protect routes that should only be accessible when authenticated
const AuthenticatedRoute = ({ children, redirectTo = '/signin' }) => {
  const { user, loading } = useBetterAuth();
  const history = useHistory();

  useEffect(() => {
    if (!loading && !user) {
      // If user is not logged in, redirect them
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

  // If user is logged in, show the children
  if (user) {
    return children;
  }

  // If user is not logged in, return null while redirect happens
  return null;
};

export default AuthenticatedRoute;