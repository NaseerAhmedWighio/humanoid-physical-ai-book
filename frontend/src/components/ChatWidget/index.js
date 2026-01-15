import React, { useState, useRef, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import './ChatWidget.css';

const ChatWidget = ({ isOpen, onClose, onToggle, initialMessage, onInitialMessageProcessed }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isMinimized, setIsMinimized] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  const { colorMode } = useColorMode();

 // ChatWidget.js mein ye useEffect replace kar do
useEffect(() => {
  if (initialMessage && isOpen) {
    // Pehle check karo ki ye message already add to nahi ho gaya
    const alreadyHasThisMessage = messages.some(
      msg => msg.sender === 'user' && msg.text === initialMessage
    );

    if (!alreadyHasThisMessage) {
      const userMessage = {
        id: Date.now(),
        text: initialMessage,
        sender: 'user',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };

      setMessages(prev => [...prev, userMessage]);
      processMessage(initialMessage).then(() => {
        if (onInitialMessageProcessed) {
          onInitialMessageProcessed();
        }
      });
    } else {
      // Agar already hai to sirf clear kar do
      if (onInitialMessageProcessed) {
        onInitialMessageProcessed();
      }
    }
  }
}, [initialMessage, isOpen, messages, onInitialMessageProcessed]);
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to process messages (both user input and selected text)
  const processMessage = async (messageText) => {
    setIsLoading(true);

    try {
      // Create a new session or use an existing one
      let sessionId = localStorage.getItem('chat_session_id');
      if (!sessionId) {
        // Safely handle process.env access for both Node.js and browser environments
        const apiBaseUrl = typeof process !== 'undefined' && process.env ? process.env.REACT_APP_API_BASE_URL : undefined;
        const baseUrl = apiBaseUrl || 'http://localhost:8000';
        const sessionResponse = await fetch(`${baseUrl}/v1/chat/sessions`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ title: "User Chat Session" }),
        });
        if (!sessionResponse.ok) {
          throw new Error(`Failed to create session: ${sessionResponse.status}`);
        }
        const sessionData = await sessionResponse.json();
        sessionId = sessionData.session_id;
        localStorage.setItem('chat_session_id', sessionId);
      }

      // Safely handle process.env access for both Node.js and browser environments
      const apiBaseUrl = typeof process !== 'undefined' && process.env ? process.env.REACT_APP_API_BASE_URL : undefined;
      const baseUrl = apiBaseUrl || 'http://localhost:8000';
      const response = await fetch(`${baseUrl}/v1/chat/sessions/${sessionId}/messages`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ content: messageText }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        sources: data.sources,
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      let errorMessageText = 'Sorry, I encountered an error. Please try again.';

      // Extract more specific error information
      if (error.message) {
        if (error.message.includes('HTTP error')) {
          errorMessageText = `HTTP Error: ${error.message}`;
        } else if (error.message.includes('Failed to create session')) {
          errorMessageText = `Session Error: ${error.message}`;
        } else {
          errorMessageText = `Error: ${error.message}`;
        }
      } else if (error.toString()) {
        errorMessageText = `Error: ${error.toString()}`;
      }

      const errorMessage = {
        id: Date.now() + 1,
        text: errorMessageText,
        sender: 'bot',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to format message text with HTML instead of markdown
  const formatMessageText = (text) => {
    if (!text) return text; // Return original if empty

    // First, escape HTML to prevent XSS
    let formattedText = text
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#x27;');

    // Replace **bold** with <strong>bold</strong>
    formattedText = formattedText.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');

    // Replace *italic* with <em>italic</em> if needed
    formattedText = formattedText.replace(/\*(.*?)\*/g, '<em>$1</em>');

    // Replace `code` with <code>code</code>
    formattedText = formattedText.replace(/`(.*?)`/g, '<code>$1</code>');

    // Replace links [text](url) with <a> tags
    formattedText = formattedText.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank" rel="noopener">$1</a>');

    // Replace markdown headers (# ## ### etc.) with HTML headers
    formattedText = formattedText.replace(/^(\s*#{1,6})\s+(.*?)\s*$/gm, (match, hashes, content) => {
      const level = Math.min(hashes.trim().length, 6);
      return `<h${level}>${content.trim()}</h${level}>`;
    });

    // Replace newlines with proper HTML structure
    // Split by double newlines first (paragraphs)
    const paragraphs = formattedText.split(/\n\s*\n/);
    formattedText = paragraphs
      .map(p => {
        // Handle single newlines within paragraphs
        let lineFormatted = p.replace(/\n/g, '<br>');
        // Only wrap in paragraph if it's not already a header or other HTML element
        if (!lineFormatted.match(/^<h[1-6]>.*<\/h[1-6]>$/)) {
          return `<p>${lineFormatted}</p>`;
        }
        return lineFormatted;
      })
      .join('');

    return formattedText;
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    processMessage(inputValue); // Use the new processMessage function
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  if (!isOpen && !isMinimized) return null;

  if (isMinimized) {
    return (
      <div className={`chat-widget ${colorMode} minimized`}>
        <div className="chat-header" onClick={() => setIsMinimized(false)}>
          <div className="chat-title">AI Assistant</div>
          <div className="chat-controls">
            <button onClick={(e) => { e.stopPropagation(); onClose(); }} className="chat-close">
              ×
            </button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className={`chat-widget ${colorMode}`}>
      <div className="chat-header">
        <div className="chat-title">AI Assistant</div>
        <div className="chat-controls">
          <button
            onClick={() => setIsMinimized(true)}
            className="chat-minimize"
            title="Minimize"
          >
            −
          </button>
          <button
            onClick={onClose}
            className="chat-close"
            title="Close"
          >
            ×
          </button>
        </div>
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="chat-welcome">
            <h3>Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics Textbook.</h3>
            <p>Ask me anything about the content, and I'll help you find relevant information.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div key={message.id} className={`chat-message ${message.sender}`}>
              <div className="message-content">
                <div className="message-text" dangerouslySetInnerHTML={{__html: formatMessageText(message.text)}}></div>
                {message.sources && message.sources.length > 0 && (
                  <div className="message-sources">
                    <details>
                      <summary>Sources</summary>
                      <ul>
                        {message.sources.map((source, index) => (
                          <li key={index}>
                            <a href={`/docs/${source.file_path}`} target="_blank" rel="noopener noreferrer">
                              {source.title || source.file_path}
                            </a>
                          </li>
                        ))}
                      </ul>
                    </details>
                  </div>
                )}
                <div className="message-timestamp">{message.timestamp}</div>
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="chat-message bot">
            <div className={`typing-indicator ${colorMode === 'dark' ? 'dark' : ''}`}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form className="chat-input-form" onSubmit={handleSubmit}>
        <textarea
          ref={inputRef}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about the textbook content..."
          disabled={isLoading}
          rows={1}
        />
        <button type="submit" disabled={isLoading || !inputValue.trim()}>
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default ChatWidget;