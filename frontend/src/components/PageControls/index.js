import React from 'react';
import PersonalizationToggle from '../PersonalizationToggle';
import TranslateButton from '../TranslateButton';
import './PageControls.css';

const PageControls = () => {
  return (
    <div className="page-controls">
      <div className="controls-wrapper">
        <div className="personalization-control">
          <PersonalizationToggle />
        </div>
        <div className="translate-control">
          <TranslateButton />
        </div>
      </div>
    </div>
  );
};

export default PageControls;