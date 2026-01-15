import React, { useState, useCallback } from 'react';
import { translationService } from '../../services/translationService';
import './TranslateButton.css';

const TranslateButton = () => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [currentLanguage, setCurrentLanguage] = useState('en');

  const handleTranslate = useCallback(async () => {
    setIsTranslating(true);

    try {
      if (currentLanguage === 'en') {
        // Translate from English to Urdu
        await translationService.translatePageContent('ur', 'en');
        setCurrentLanguage('ur');
      } else {
        // Revert from Urdu to English
        translationService.revertTranslation();
        setCurrentLanguage('en');
      }
    } catch (error) {
      console.error('Translation error:', error);
    } finally {
      setIsTranslating(false);
    }
  }, [currentLanguage]);

  return (
    <div className="translate-button-wrapper">
      <button
        className={`translate-button ${currentLanguage === 'ur' ? 'active' : ''}`}
        onClick={handleTranslate}
        disabled={isTranslating}
        title={currentLanguage === 'ur' ? 'Switch to English' : 'Translate to Urdu'}
      >
        {isTranslating ? (
          <span className="loading">🔄</span>
        ) : (
          <span className="lang-icon">{currentLanguage === 'ur' ? '🇺🇸' : '🇵🇰'}</span>
        )}
        <span className="lang-text">{currentLanguage === 'en' ? 'UR' : 'EN'}</span>
      </button>
    </div>
  );
};

export default TranslateButton;