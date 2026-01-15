import React, { useState } from 'react';
import { useBetterAuth } from '../../context/BetterAuthContext';
import { useHistory } from '@docusaurus/router';
import ProtectedRoute from './ProtectedRoute';
import './Auth.css';

const Signin = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const { login } = useBetterAuth();
  const history = useHistory();

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      const result = await login(email, password);

      if (result.success) {
        // Redirect to homepage after successful login
        history.push('/');
      } else {
        setError(result.error || 'Login failed');
      }
    } catch (err) {
      setError('An error occurred during login');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <ProtectedRoute redirectTo="/">
      <div className="auth-container">
        <div className="auth-form auth-form--signin">
          <div className="auth-form__header">
            <h1>Sign In</h1>
            <p className="auth-form__subtitle">Welcome back! Please sign in to continue</p>
          </div>

          {error && <div className="auth-form__error">{error}</div>}

          <form onSubmit={handleSubmit} className="auth-form__form">
            <div className="auth-form__group">
              <label htmlFor="email" className="auth-form__label">Email</label>
              <input
                type="email"
                id="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
                className="auth-form__input"
                placeholder="Enter your email"
              />
            </div>

            <div className="auth-form__group">
              <label htmlFor="password" className="auth-form__label">Password</label>
              <input
                type="password"
                id="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                className="auth-form__input"
                placeholder="Enter your password"
              />
            </div>

            <button type="submit" disabled={loading} className="auth-form__submit">
              {loading ? (
                <span className="auth-form__loading">
                  <span className="auth-form__spinner"></span>
                  Signing In...
                </span>
              ) : (
                'Sign In'
              )}
            </button>
          </form>

          <div className="auth-form__footer">
            <p className="auth-form__text">
              Don't have an account?{' '}
              <a href="/signup" className="auth-form__link">Sign up</a>
            </p>
          </div>
        </div>
      </div>
    </ProtectedRoute>
  );
};

export default Signin;