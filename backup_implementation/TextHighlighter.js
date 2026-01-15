import React, { useEffect, useRef } from 'react';

const TextHighlighter = ({ onTextSelect, onTextAsk }) => {
  const contentRef = useRef(null);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection.toString().trim();

      if (selectedText) {
        // Get the range of the selection to understand context
        const range = selection.getRangeAt(0);
        const selectedElement = range.commonAncestorContainer;

        // Ensure we're getting text from the content area
        if (contentRef.current && contentRef.current.contains(selectedElement)) {
          // Trigger the text selection callback
          if (onTextSelect) {
            onTextSelect({
              text: selectedText,
              element: selectedElement,
              range: range,
              position: {
                x: range.getBoundingClientRect().left,
                y: range.getBoundingClientRect().top
              }
            });
          }
        }
      }
    };

    // Add event listeners for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', (e) => {
      if (e.key === 'Escape') {
        window.getSelection().removeAllRanges();
      }
    });

    // Cleanup event listeners
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', (e) => {
        if (e.key === 'Escape') {
          window.getSelection().removeAllRanges();
        }
      });
    };
  }, [onTextSelect]);

  // Function to manually select text from the entire page
  const selectAllText = () => {
    const selection = window.getSelection();
    if (selection) {
      selection.removeAllRanges();

      const range = document.createRange();
      range.selectNodeContents(contentRef.current || document.body);
      selection.addRange(range);

      const selectedText = selection.toString().trim();
      if (selectedText && onTextSelect) {
        onTextSelect({
          text: selectedText,
          element: contentRef.current || document.body,
          range: range,
          position: { x: 0, y: 0 }
        });
      }
    }
  };

  // Function to ask about selected text
  const askAboutSelection = () => {
    const selection = window.getSelection();
    const selectedText = selection.toString().trim();

    if (selectedText && onTextAsk) {
      onTextAsk(selectedText);
    } else {
      alert('Please select some text first.');
    }
  };

  return (
    <div className="text-highlighter-container">
      <div
        ref={contentRef}
        className="content-area"
        onMouseUp={() => {
          // This will trigger the selection handler in useEffect
        }}
      >
        {/* The content area where text selection will be enabled */}
        <div className="selection-instructions">
          <p>Select any text on this page to highlight it and use the options below.</p>
        </div>
      </div>

      <div className="selection-controls">
        <button
          className="ask-button"
          onClick={askAboutSelection}
          disabled={!window.getSelection || !window.getSelection().toString().trim()}
        >
          Ask About Selected Text
        </button>

        <button
          className="select-all-button"
          onClick={selectAllText}
        >
          Select All Text
        </button>
      </div>
    </div>
  );
};

export default TextHighlighter;