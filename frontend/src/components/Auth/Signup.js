import React, { useState } from 'react';
import { useBetterAuth } from '../../context/BetterAuthContext';
import { useHistory } from '@docusaurus/router';
import ProtectedRoute from './ProtectedRoute';
import './Auth.css';

const Signup = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [selectedPreferences, setSelectedPreferences] = useState({
    has_mobile: false,
    has_laptop: false,
    has_physical_robot: false,
    has_other_hardware: '',
    web_dev_experience: 'beginner'
  });
  const { register } = useBetterAuth(); // Using enhanced register function
  const history = useHistory();

  const handleCheckboxChange = (field) => (e) => {
    setSelectedPreferences(prev => ({
      ...prev,
      [field]: e.target.checked
    }));
  };

  const handleTextChange = (field) => (e) => {
    setSelectedPreferences(prev => ({
      ...prev,
      [field]: e.target.value
    }));
  };

  const handleExperienceChange = (e) => {
    setSelectedPreferences(prev => ({
      ...prev,
      web_dev_experience: e.target.value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    // Password validation
    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    // Password strength validation
    const passwordValidation = validatePassword(password);
    if (!passwordValidation.isValid) {
      setError(passwordValidation.message);
      return;
    }

    setLoading(true);
    setError('');

    try {
      // Use the enhanced register function that handles combined registration
      const result = await register(email, password, selectedPreferences);

      if (result.success) {
        // Redirect to homepage after successful registration with preferences
        history.push('/');
        return location.pathname('/');
      } else {
        setError(result.error || 'Registration failed');
      }
    } catch (err) {
      setError('An error occurred during registration');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  const validatePassword = (password) => {
    // Check minimum length (at least 8 characters)
    if (password.length < 8) {
      return {
        isValid: false,
        message: 'Password must be at least 8 characters long'
      };
    }

    // Check maximum length to prevent bcrypt 72-byte limit issues
    if (password.length > 70) {
      return {
        isValid: false,
        message: 'Password must be no more than 70 characters long to comply with security requirements'
      };
    }

    // Check for at least one uppercase letter
    if (!/[A-Z]/.test(password)) {
      return {
        isValid: false,
        message: 'Password must contain at least one uppercase letter'
      };
    }

    // Check for at least one lowercase letter
    if (!/[a-z]/.test(password)) {
      return {
        isValid: false,
        message: 'Password must contain at least one lowercase letter'
      };
    }

    // Check for at least one number
    if (!/\d/.test(password)) {
      return {
        isValid: false,
        message: 'Password must contain at least one number'
      };
    }

    // Check for at least one special character
    if (!/[!@#$%^&*()_+\-=\[\]{};':"\\|,.<>\/?]/.test(password)) {
      return {
        isValid: false,
        message: 'Password must contain at least one special character (!@#$%^&*()_+-=[]{}|;:,.<>?)'
      };
    }

    return {
      isValid: true,
      message: ''
    };
  };

  return (
    <ProtectedRoute redirectTo="/">
      <div className="auth-container">
        <div className="auth-form auth-form--signup">
          <div className="auth-form__header">
            <h1>Create Account</h1>
            <p className="auth-form__subtitle">Join us to access personalized content and features</p>
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
                placeholder="Create a strong password"
              />
            </div>

            <div className="auth-form__group">
              <label htmlFor="confirmPassword" className="auth-form__label">Confirm Password</label>
              <input
                type="password"
                id="confirmPassword"
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                required
                className="auth-form__input"
                placeholder="Confirm your password"
              />
            </div>

            {/* Hardware Preferences Section */}
            <div className="auth-form__group">
              <h3 style={{ marginBottom: '1rem', fontSize: '1.1rem', color: 'var(--ifm-heading-color)' }}>
                Hardware Available
              </h3>
              <div className="checkbox-group" style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
                <label className="checkbox-option" style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer' }}>
                  <input
                    type="checkbox"
                    checked={selectedPreferences.has_mobile}
                    onChange={handleCheckboxChange('has_mobile')}
                    style={{ width: 'auto', marginRight: '0.5rem' }}
                  />
                  <span style={{ color: 'var(--ifm-font-color-base)' }}>Mobile Device</span>
                </label>

                <label className="checkbox-option" style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer' }}>
                  <input
                    type="checkbox"
                    checked={selectedPreferences.has_laptop}
                    onChange={handleCheckboxChange('has_laptop')}
                    style={{ width: 'auto', marginRight: '0.5rem' }}
                  />
                  <span style={{ color: 'var(--ifm-font-color-base)' }}>Laptop/Computer</span>
                </label>

                <label className="checkbox-option" style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer' }}>
                  <input
                    type="checkbox"
                    checked={selectedPreferences.has_physical_robot}
                    onChange={handleCheckboxChange('has_physical_robot')}
                    style={{ width: 'auto', marginRight: '0.5rem' }}
                  />
                  <span style={{ color: 'var(--ifm-font-color-base)' }}>Physical Robot</span>
                </label>
              </div>
            </div>

            <div className="auth-form__group">
              <label htmlFor="other_hardware" className="auth-form__label">
                Other Hardware (Optional)
              </label>
              <input
                type="text"
                id="other_hardware"
                value={selectedPreferences.has_other_hardware}
                onChange={handleTextChange('has_other_hardware')}
                className="auth-form__input"
                placeholder="Describe any other hardware you have access to"
              />
            </div>

            <div className="auth-form__group">
              <label htmlFor="experience" className="auth-form__label">
                Web Development Experience
              </label>
              <select
                id="experience"
                value={selectedPreferences.web_dev_experience}
                onChange={handleExperienceChange}
                className="auth-form__input"
                style={{
                  padding: '0.875rem 1rem',
                  cursor: 'pointer'
                }}
              >
                <option value="beginner">Beginner (0-1 years)</option>
                <option value="intermediate">Intermediate (1-3 years)</option>
                <option value="experienced">Experienced (3-5 years)</option>
                <option value="expert">Expert (5+ years)</option>
              </select>
            </div>

            <button type="submit" disabled={loading} className="auth-form__submit">
              {loading ? (
                <span className="auth-form__loading">
                  <span className="auth-form__spinner"></span>
                  Creating Account...
                </span>
              ) : (
                'Create Account & Continue'
              )}
            </button>
          </form>

          <div className="auth-form__footer">
            <p className="auth-form__text">
              Already have an account?{' '}
              <a href="/signin" className="auth-form__link">Sign in</a>
            </p>
          </div>
        </div>
      </div>
    </ProtectedRoute>
  );
};

export default Signup;