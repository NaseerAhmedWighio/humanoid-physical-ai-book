import React from 'react';
import { useUserPreference } from '../context/UserPreferenceContext';

const PersonalizationToggle = () => {
  const {
    personalizationEnabled,
    togglePersonalization,
    setHardwarePreference,
    hardwarePreference,
    loading,
    error
  } = useUserPreference();

  const handleToggle = () => {
    if (!loading) {
      togglePersonalization();
    }
  };

  const handlePreferenceChange = (event) => {
    if (!loading) {
      setHardwarePreference(event.target.value);
    }
  };

  return (
    <div className="personalization-toggle-container">
      <div className="personalization-toggle">
        <label className="toggle-label">
          <input
            type="checkbox"
            checked={personalizationEnabled}
            onChange={handleToggle}
            disabled={loading}
            className="toggle-checkbox"
          />
          <span className="toggle-slider"></span>
          <span className="toggle-text">
            {personalizationEnabled ? 'Personalization On' : 'Personalization Off'}
          </span>
        </label>
      </div>

      {personalizationEnabled && (
        <div className="hardware-preference-selector">
          <label htmlFor="hardwarePreference">Hardware Preference:</label>
          <select
            id="hardwarePreference"
            value={hardwarePreference}
            onChange={handlePreferenceChange}
            disabled={loading}
            className="hardware-select"
          >
            <option value="mobile">Mobile Device</option>
            <option value="laptop">Laptop/Computer</option>
            <option value="physical_robot">Physical Robot</option>
          </select>
        </div>
      )}

      {error && (
        <div className="error-message">
          Error: {error}
        </div>
      )}

      {loading && (
        <div className="loading-indicator">
          Updating preferences...
        </div>
      )}
    </div>
  );
};

export default PersonalizationToggle;