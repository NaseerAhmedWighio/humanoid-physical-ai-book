import React, { useState, useEffect } from 'react';
import { useBetterAuth } from '../../context/BetterAuthContext';
import './UserProfile.css';

const UserProfile = () => {
  const { user, updateUserPreferences, loading: authLoading } = useBetterAuth();
  const [profileData, setProfileData] = useState({
    has_mobile: false,
    has_laptop: false,
    has_physical_robot: false,
    has_other_hardware: '',
    web_dev_experience: 'beginner',
    language_preference: 'en',
    personalization_enabled: true
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');

  // Initialize form with user data when user is loaded
  useEffect(() => {
    if (user) {
      setProfileData({
        has_mobile: user.has_mobile || false,
        has_laptop: user.has_laptop || false,
        has_physical_robot: user.has_physical_robot || false,
        has_other_hardware: user.has_other_hardware || '',
        web_dev_experience: user.web_dev_experience || 'beginner',
        language_preference: user.language_preference || 'en',
        personalization_enabled: user.personalization_enabled !== undefined ? user.personalization_enabled : true
      });
    }
  }, [user]);

  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;
    setProfileData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');
    setSuccess('');

    try {
      const result = await updateUserPreferences(profileData);
      if (result.success) {
        setSuccess('Profile updated successfully!');
        // Clear success message after 3 seconds
        setTimeout(() => setSuccess(''), 3000);
      } else {
        setError(result.error || 'Failed to update profile');
      }
    } catch (err) {
      setError('An error occurred while updating your profile');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  if (authLoading) {
    return (
      <div className="user-profile-container">
        <div className="loading">Loading...</div>
      </div>
    );
  }

  if (!user) {
    return (
      <div className="user-profile-container">
        <div className="error">Please sign in to view your profile.</div>
      </div>
    );
  }

  return (
    <div className="user-profile-container">
      <div className="user-profile-header">
        <h1>User Profile</h1>
        <p>Manage your account preferences and hardware settings</p>
      </div>

      <form onSubmit={handleSubmit} className="user-profile-form">
        {error && <div className="error-message">{error}</div>}
        {success && <div className="success-message">{success}</div>}

        <div className="form-section">
          <h2>Account Information</h2>
          <div className="form-group">
            <label>Email:</label>
            <input
              type="email"
              value={user.email || ''}
              disabled
              className="disabled-input"
            />
          </div>
        </div>

        <div className="form-section">
          <h2>Hardware Preferences</h2>
          <p>Select the hardware you have access to for personalized content:</p>

          <div className="checkbox-group">
            <label className="checkbox-option">
              <input
                type="checkbox"
                name="has_mobile"
                checked={profileData.has_mobile}
                onChange={handleChange}
              />
              <span>Mobile Device</span>
            </label>

            <label className="checkbox-option">
              <input
                type="checkbox"
                name="has_laptop"
                checked={profileData.has_laptop}
                onChange={handleChange}
              />
              <span>Laptop/Computer</span>
            </label>

            <label className="checkbox-option">
              <input
                type="checkbox"
                name="has_physical_robot"
                checked={profileData.has_physical_robot}
                onChange={handleChange}
              />
              <span>Physical Robot</span>
            </label>
          </div>

          <div className="form-group">
            <label htmlFor="has_other_hardware">Other Hardware:</label>
            <input
              type="text"
              id="has_other_hardware"
              name="has_other_hardware"
              value={profileData.has_other_hardware}
              onChange={handleChange}
              placeholder="Describe any other hardware you have access to"
            />
          </div>
        </div>

        <div className="form-section">
          <h2>Experience Level</h2>
          <div className="radio-group">
            <label className="radio-option">
              <input
                type="radio"
                name="web_dev_experience"
                value="beginner"
                checked={profileData.web_dev_experience === 'beginner'}
                onChange={handleChange}
              />
              <span>Beginner (0-1 years)</span>
            </label>

            <label className="radio-option">
              <input
                type="radio"
                name="web_dev_experience"
                value="intermediate"
                checked={profileData.web_dev_experience === 'intermediate'}
                onChange={handleChange}
              />
              <span>Intermediate (1-3 years)</span>
            </label>

            <label className="radio-option">
              <input
                type="radio"
                name="web_dev_experience"
                value="experienced"
                checked={profileData.web_dev_experience === 'experienced'}
                onChange={handleChange}
              />
              <span>Experienced (3-5 years)</span>
            </label>

            <label className="radio-option">
              <input
                type="radio"
                name="web_dev_experience"
                value="expert"
                checked={profileData.web_dev_experience === 'expert'}
                onChange={handleChange}
              />
              <span>Expert (5+ years)</span>
            </label>
          </div>
        </div>

        <div className="form-section">
          <h2>Content Preferences</h2>

          <div className="checkbox-group">
            <label className="checkbox-option">
              <input
                type="checkbox"
                name="personalization_enabled"
                checked={profileData.personalization_enabled}
                onChange={handleChange}
              />
              <span>Enable content personalization based on hardware preferences</span>
            </label>
          </div>

          <div className="form-group">
            <label htmlFor="language_preference">Preferred Language:</label>
            <select
              id="language_preference"
              name="language_preference"
              value={profileData.language_preference}
              onChange={handleChange}
            >
              <option value="en">English</option>
              <option value="ur">Urdu</option>
            </select>
          </div>
        </div>

        <button type="submit" disabled={loading} className="submit-btn">
          {loading ? 'Saving...' : 'Save Changes'}
        </button>
      </form>
    </div>
  );
};

export default UserProfile;