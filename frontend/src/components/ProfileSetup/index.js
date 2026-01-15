import React, { useState, useEffect } from 'react';
import { useBetterAuth } from '../../context/BetterAuthContext';
import './ProfileSetup.css';

const ProfileSetup = ({ onComplete, isUpdate = false }) => {
  const { user, updateUserPreferences } = useBetterAuth();
  const [profileData, setProfileData] = useState({
    has_mobile: false,
    has_laptop: false,
    has_physical_robot: false,
    has_other_hardware: '',
    web_dev_experience: '',
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  // Initialize form with user data if available (for updates)
  useEffect(() => {
    if (user && isUpdate) {
      setProfileData({
        has_mobile: user.has_mobile || false,
        has_laptop: user.has_laptop || false,
        has_physical_robot: user.has_physical_robot || false,
        has_other_hardware: user.has_other_hardware || '',
        web_dev_experience: user.web_dev_experience || '',
      });
    }
  }, [user, isUpdate]);

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

    try {
      if (isUpdate) {
        // Update existing user preferences
        const result = await updateUserPreferences(profileData);
        if (result.success) {
          if (onComplete) {
            onComplete(result.user);
          }
        } else {
          setError(result.error);
        }
      } else {
        // Complete profile setup for new user
        if (onComplete) {
          onComplete(profileData);
        }
      }
    } catch (err) {
      setError('An error occurred while saving your profile');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="profile-setup-modal">
      <div className="profile-setup-content">
        <h2>{isUpdate ? 'Update Your Profile' : 'Complete Your Profile'}</h2>
        <p>Help us customize your learning experience</p>

        <form onSubmit={handleSubmit} className="profile-setup-form">
          <div className="form-section">
            <h3>Hardware Available</h3>
            <div className="checkbox-group">
              <label>
                <input
                  type="checkbox"
                  name="has_mobile"
                  checked={profileData.has_mobile}
                  onChange={handleChange}
                />
                Mobile Device
              </label>
              <label>
                <input
                  type="checkbox"
                  name="has_laptop"
                  checked={profileData.has_laptop}
                  onChange={handleChange}
                />
                Laptop/Computer
              </label>
              <label>
                <input
                  type="checkbox"
                  name="has_physical_robot"
                  checked={profileData.has_physical_robot}
                  onChange={handleChange}
                />
                Physical Robot
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
                placeholder="Describe any other hardware you have"
              />
            </div>
          </div>

          <div className="form-section">
            <h3>Web Development Experience</h3>
            <div className="radio-group">
              <label>
                <input
                  type="radio"
                  name="web_dev_experience"
                  value="beginner"
                  checked={profileData.web_dev_experience === 'beginner'}
                  onChange={handleChange}
                />
                Beginner
              </label>
              <label>
                <input
                  type="radio"
                  name="web_dev_experience"
                  value="intermediate"
                  checked={profileData.web_dev_experience === 'intermediate'}
                  onChange={handleChange}
                />
                Intermediate
              </label>
              <label>
                <input
                  type="radio"
                  name="web_dev_experience"
                  value="experienced"
                  checked={profileData.web_dev_experience === 'experienced'}
                  onChange={handleChange}
                />
                Experienced
              </label>
              <label>
                <input
                  type="radio"
                  name="web_dev_experience"
                  value="expert"
                  checked={profileData.web_dev_experience === 'expert'}
                  onChange={handleChange}
                />
                Expert
              </label>
            </div>
          </div>

          {error && <div className="error-message">{error}</div>}

          <button type="submit" disabled={loading} className="submit-btn">
            {loading ? (isUpdate ? 'Updating...' : 'Saving...') : (isUpdate ? 'Update Profile' : 'Save Profile')}
          </button>
        </form>
      </div>
    </div>
  );
};

export default ProfileSetup;