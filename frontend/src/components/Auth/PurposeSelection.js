import React, { useState, useEffect } from 'react';
import { useBetterAuth } from '../../context/BetterAuthContext';
import { useHistory } from '@docusaurus/router';
import AuthenticatedRoute from './AuthenticatedRoute';
import './Auth.css';

const PurposeSelection = () => {
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState('');
  const [selectedPreferences, setSelectedPreferences] = useState({
    has_mobile: false,
    has_laptop: false,
    has_physical_robot: false,
    has_other_hardware: '',
    web_dev_experience: 'beginner'
  });
  const { user, updateUserPreferences } = useBetterAuth();
  const history = useHistory();

  useEffect(() => {
    if (user) {
      // Pre-populate with existing user data if available
      setSelectedPreferences({
        has_mobile: user.has_mobile || false,
        has_laptop: user.has_laptop || false,
        has_physical_robot: user.has_physical_robot || false,
        has_other_hardware: user.has_other_hardware || '',
        web_dev_experience: user.web_dev_experience || 'beginner'
      });
      setLoading(false);
    }
  }, [user]);

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
    setLoading(true);
    setError('');

    try {
      const result = await updateUserPreferences(selectedPreferences);

      if (result.success) {
        // Redirect to homepage after successful update
        history.push('/');
      } else {
        setError(result.error || 'Failed to update preferences');
      }
    } catch (err) {
      setError('An error occurred while updating preferences');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  if (loading && user) {
    return (
      <div className="auth-container">
        <div className="auth-form">
          <div className="auth-form__loading">
            <span className="auth-form__spinner"></span>
            Loading...
          </div>
        </div>
      </div>
    );
  }

  return (
    <AuthenticatedRoute redirectTo="/signin">
      <div className="auth-container">
        <div className="auth-form auth-form--signup">
          <div className="auth-form__header">
            <h1>Tell Us About Yourself</h1>
            <p className="auth-form__subtitle">Help us personalize your learning experience</p>
          </div>

          {error && <div className="auth-form__error">{error}</div>}

          <form onSubmit={handleSubmit} className="auth-form__form">
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
                  Saving Preferences...
                </span>
              ) : (
                'Save & Continue'
              )}
            </button>
          </form>

          <div className="auth-form__footer">
            <p className="auth-form__text">
              You can update these preferences anytime in your profile settings.
            </p>
          </div>
        </div>
      </div>
    </AuthenticatedRoute>
  );
};

export default PurposeSelection;