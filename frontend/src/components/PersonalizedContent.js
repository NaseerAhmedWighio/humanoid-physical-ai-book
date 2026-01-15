import React, { useState, useEffect } from 'react';
import { useUserPreference } from '../context/UserPreferenceContext';
import axios from 'axios';
import { API_BASE_URL } from '../constants/api';
import Notification from './Notification';

const PersonalizedContent = ({ originalContent, contentId }) => {
  const { hardwarePreference, personalizationEnabled } = useUserPreference();
  const [personalizedContent, setPersonalizedContent] = useState(originalContent);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [showNotification, setShowNotification] = useState(false);
  const [notificationMessage, setNotificationMessage] = useState('');
  const [notificationType, setNotificationType] = useState('info');

  useEffect(() => {
    if (personalizationEnabled && contentId) {
      // Apply personalization based on hardware preference
      setIsLoading(true);
      setError(null);

      // Try to get personalized content from backend API
      const fetchPersonalizedContent = async () => {
        try {
          const response = await axios.get(`${API_BASE_URL}/personalization/content/${contentId}`, {
            params: {
              hardware_preference: hardwarePreference,
              user_id: 'current_user' // This would be the actual user ID in a real implementation
            }
          });

          if (response.data.error) {
            // Show notification about service unavailability
            setNotificationMessage('Personalization service is temporarily unavailable. Showing original content.');
            setNotificationType('warning');
            setShowNotification(true);
            setPersonalizedContent(originalContent);
          } else {
            setPersonalizedContent(response.data.content);
          }
        } catch (apiError) {
          console.error('Error fetching personalized content:', apiError);

          // Fallback to client-side personalization if API fails
          console.warn('Falling back to client-side personalization');
          const fallbackContent = applyClientSidePersonalization();
          setPersonalizedContent(fallbackContent);

          // Show notification about the fallback
          setNotificationMessage('Personalization service unavailable. Using local personalization.');
          setNotificationType('warning');
          setShowNotification(true);
        } finally {
          setIsLoading(false);
        }
      };

      // Client-side personalization fallback
      const applyClientSidePersonalization = () => {
        let content = originalContent;

        if (hardwarePreference === 'mobile') {
          // Mobile-specific optimizations
          content = `<!-- Mobile-optimized content -->\n<div class="mobile-content">\n${originalContent}\n</div>`;
        } else if (hardwarePreference === 'laptop') {
          // Desktop-specific optimizations
          content = `<!-- Desktop-optimized content -->\n<div class="desktop-content">\n${originalContent}\n</div>`;
        } else if (hardwarePreference === 'physical_robot') {
          // Physical robot-specific content
          content = `<!-- Physical Robot Content -->\n<div class="robot-content">\n${originalContent}\n</div>`;
        }

        return content;
      };

      fetchPersonalizedContent();
    } else {
      // Reset to original content when personalization is disabled
      setPersonalizedContent(originalContent);
      setIsLoading(false);
      setError(null);
    }
  }, [originalContent, personalizationEnabled, hardwarePreference, contentId]);

  const handleNotificationClose = () => {
    setShowNotification(false);
  };

  if (isLoading) {
    return (
      <div>
        <div className="loading-content">Personalizing content...</div>
        {showNotification && (
          <Notification
            message={notificationMessage}
            type={notificationType}
            onClose={handleNotificationClose}
          />
        )}
      </div>
    );
  }

  return (
    <div>
      {showNotification && (
        <Notification
          message={notificationMessage}
          type={notificationType}
          onClose={handleNotificationClose}
        />
      )}
      <div
        className={`personalized-content-container ${personalizationEnabled ? 'personalized' : 'original'}`}
        dangerouslySetInnerHTML={{ __html: personalizedContent }}
      />
    </div>
  );
};

export default PersonalizedContent;