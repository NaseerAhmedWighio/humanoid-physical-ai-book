import React, { useEffect, useRef } from 'react';

const TextHighlighter = ({ onSelectionAsk }) => {
  const tooltipRef = useRef(null);
  const [showTooltip, setShowTooltip] = React.useState(false);
  const [tooltipPosition, setTooltipPosition] = React.useState({ x: 0, y: 0 });
  const [selectedText, setSelectedText] = React.useState('');

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setTooltipPosition({
          x: rect.left + window.scrollX,
          y: rect.top + window.scrollY - 40  // Position above the selection
        });

        setSelectedText(text);
        setShowTooltip(true);
      } else {
        setShowTooltip(false);
      }
    };

    const handleClickOutside = (event) => {
      if (tooltipRef.current && !tooltipRef.current.contains(event.target)) {
        setShowTooltip(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('click', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('click', handleClickOutside);
    };
  }, []);

  const handleAskClick = () => {
    if (selectedText && onSelectionAsk) {
      onSelectionAsk(selectedText);
      setShowTooltip(false);
    }
  };

  if (!showTooltip || !selectedText) {
    return null;
  }

  return (
    <div
      ref={tooltipRef}
      className="text-highlighter-tooltip"
      style={{
        position: 'absolute',
        left: `${tooltipPosition.x}px`,
        top: `${tooltipPosition.y}px`,
        zIndex: 10000,
        backgroundColor: '#007bff',
        color: 'white',
        padding: '8px 16px',
        borderRadius: '6px',
        fontSize: '14px',
        fontWeight: 'bold',
        boxShadow: '0 4px 12px rgba(0,0,0,0.3), 0 2px 4px rgba(0,0,0,0.2)',
        transform: 'translateY(-10px)',
        opacity: 0,
        animation: 'fadeInSlideUp 0.2s forwards',
        minWidth: '120px',
        textAlign: 'center',
        pointerEvents: 'auto' // Ensure the tooltip receives clicks
      }}
    >
      <style>{`
        @keyframes fadeInSlideUp {
          from {
            opacity: 0;
            transform: translateY(-10px);
          }
          to {
            opacity: 1;
            transform: translateY(0);
          }
        }
        .text-highlighter-tooltip {
          font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
        }
        .text-highlighter-tooltip button {
          background: transparent;
          border: none;
          color: white;
          cursor: pointer;
          font-size: 14px;
          font-weight: bold;
          padding: 0;
          margin: 0;
          text-decoration: underline;
          outline: none;
          transition: opacity 0.2s ease;
        }
        .text-highlighter-tooltip button:hover {
          opacity: 0.9;
        }
        .text-highlighter-tooltip button:active {
          opacity: 0.8;
        }
      `}</style>
      <button
        onClick={handleAskClick}
        onMouseDown={(e) => e.preventDefault()} // Prevent blur on click
        title="Ask AI about selected text"
      >
        Ask Question
      </button>
    </div>
  );
};

export default TextHighlighter;