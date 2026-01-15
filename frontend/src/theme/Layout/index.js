import React, { useState, useEffect } from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatkit from '@site/src/components/Chatkit';
import TextHighlighter from '@site/src/components/TextHighlighter';
import PageControls from '@site/src/components/PageControls';
// import PersonalizedContentTrigger from '@site/src/components/PersonalizedContentTrigger';
import PersonalizationToggle from '../../components/PersonalizationToggle';
import { useLocation } from '@docusaurus/router';
import { ThemeProvider } from '@site/src/context/ThemeContext';
import { BetterAuthProvider } from '@site/src/context/BetterAuthContext';
import { PersonalizationProvider } from '@site/src/context/PersonalizationContext';
import { UserPreferenceProvider } from '@site/src/context/UserPreferenceContext';
import { translationService } from '@site/src/services/translationService';

export default function LayoutWrapper(props) {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [initialMessage, setInitialMessage] = useState(null);
  const location = useLocation();
  const [shouldShowPersonalizedTrigger, setShouldShowPersonalizedTrigger] = useState(false);

  useEffect(() => {
    const isHomePage = location.pathname === '/' || location.pathname === '/index.html';
    const isAuthPage = location.pathname.includes('/auth/') ||
                      location.pathname.includes('/signin') ||
                      location.pathname.includes('/signup') ||
                      location.pathname.includes('/signout');
    const is404Page = location.pathname === '/404.html' || location.pathname.includes('/404');
    const isDocsIntroPage = location.pathname.startsWith('/docs/intro');

    // Personalized trigger sirf in pages par show karo
    setShouldShowPersonalizedTrigger(!isHomePage && !isAuthPage && !is404Page && !isDocsIntroPage);
  }, [location]);

  // TextHighlighter sirf auth pages par nahi dikhana
  const shouldShowTextHighlighter = !(
    location.pathname.includes('/auth/') ||
    location.pathname.includes('/signin') ||
    location.pathname.includes('/signup') ||
    location.pathname.includes('/signout')
  );

  const handleChatToggle = () => {
    setIsChatOpen(prev => !prev);
  };

  const handleChatClose = () => {
    setIsChatOpen(false);
  };

  const handleSelectionAsk = (selectedText) => {
    if (!selectedText || selectedText.trim() === '') return;

    setInitialMessage(selectedText.trim());
    setIsChatOpen(true);
  };

  // ChatWidget ne message process kar liya to initialMessage clear kar do
  const handleInitialMessageProcessed = () => {
    setInitialMessage(null);
  };


  return (
    <BetterAuthProvider>
      <PersonalizationProvider>
        <UserPreferenceProvider>
          <ThemeProvider>
            <>
              <OriginalLayout {...props}>
                {/* Main content without the top controls which are now in navbar */}
                <div className="main-content-wrapper">
                  {props.children}
                </div>

                {/* Text Highlighter - selected text par "Ask Question" button dikhayega */}
                {shouldShowTextHighlighter && (
                  <TextHighlighter onSelectionAsk={handleSelectionAsk} />
                )}

                {/* Chatkit Widget */}
                <Chatkit
                  isOpen={isChatOpen}
                  onClose={handleChatClose}
                  initialMessage={initialMessage}
                  onInitialMessageProcessed={handleInitialMessageProcessed}
                />
              </OriginalLayout>

              {/* Floating Chat Button - jab chat closed ho */}
              {!isChatOpen && (
                <button
                  onClick={handleChatToggle}
                  className="floating-chat-button"
                  aria-label="Open AI Chat Assistant"
                  style={{
                    position: 'fixed',
                    bottom: '20px',
                    right: '20px',
                    width: '60px',
                    height: '60px',
                    borderRadius: '50%',
                    backgroundColor: '#007bff',
                    color: 'white',
                    border: 'none',
                    fontSize: '28px',
                    cursor: 'pointer',
                    boxShadow: '0 4px 15px rgba(0, 0, 0, 0.3)',
                    zIndex: '1000',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    transition: 'all 0.3s ease',
                  }}
                  onMouseOver={(e) => e.currentTarget.style.transform = 'scale(1.1)'}
                  onMouseOut={(e) => e.currentTarget.style.transform = 'scale(1)'}
                >
                  💬
                </button>
              )}
            </>
          </ThemeProvider>
        </UserPreferenceProvider>
      </PersonalizationProvider>
    </BetterAuthProvider>
  );
}