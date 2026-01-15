import React, { useState, useEffect, useRef } from 'react';
import { getItem, setItem, removeItem } from '../services/localStorage';
import { useUserPreference } from '../context/UserPreferenceContext';
import { getSelectedText } from '../services/contentSelectionService';
import axios from 'axios';
import { API_BASE_URL } from '../constants/api';
import Notification from './Notification';

const ChatWidget = ({ isOpen = false, onClose, onToggle, initialMessage = null, onInitialMessageProcessed }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [showNotification, setShowNotification] = useState(false);
  const [notificationMessage, setNotificationMessage] = useState('');
  const [notificationType, setNotificationType] = useState('info');
  const messagesEndRef = useRef(null);
  const { personalizationEnabled, hardwarePreference } = useUserPreference();

  // Conversation ID for persistence
  const [conversationId, setConversationId] = useState(null);

  // Load conversation from localStorage on component mount and handle initial message
  useEffect(() => {
    try {
      const savedConversation = getItem('chat_conversation');
      if (savedConversation) {
        setMessages(savedConversation.messages || []);
        setConversationId(savedConversation.conversationId);
      } else {
        // Create a new conversation ID if none exists (match backend format)
        const newId = `session_${Date.now()}`;
        setConversationId(newId);
        setItem('chat_conversation', {
          conversationId: newId,
          messages: [],
          createdAt: new Date().toISOString()
        });
      }

      // Handle initial message if provided
      if (initialMessage) {
        handleSendMessage(initialMessage);
        if (onInitialMessageProcessed) {
          onInitialMessageProcessed();
        }
      }
    } catch (storageError) {
      console.error('Error loading conversation from localStorage:', storageError);
      // Fallback: create new conversation
      const newId = `conv_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      setConversationId(newId);
      setMessages([]);

      // Show notification about localStorage issue
      setNotificationMessage('Failed to load conversation from storage. Starting new conversation.');
      setNotificationType('warning');
      setShowNotification(true);
    }
  }, [initialMessage, onInitialMessageProcessed]);

  // Save conversation to localStorage whenever messages change
  useEffect(() => {
    try {
      if (conversationId) {
        const conversationData = {
          conversationId,
          messages,
          updatedAt: new Date().toISOString(),
          personalizationEnabled,
          hardwarePreference
        };
        const saveSuccess = setItem('chat_conversation', conversationData);
        if (!saveSuccess) {
          console.warn('Failed to save conversation to localStorage');

          // Show notification about save failure
          setNotificationMessage('Failed to save conversation to storage.');
          setNotificationType('warning');
          setShowNotification(true);
        }
      }
    } catch (storageError) {
      console.error('Error saving conversation to localStorage:', storageError);
    }
  }, [messages, conversationId, personalizationEnabled, hardwarePreference]);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async (messageText = null) => {
    const textToSend = messageText || inputValue;

    if (!textToSend.trim() || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: textToSend,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);

    if (!messageText) {
      // Only clear input if we're not sending a selected text
      setInputValue('');
    }

    setIsLoading(true);
    setError(null);

    try {
      // Try to send message to backend API
      let botMessage;
      try {
        const response = await axios.post(`${API_BASE_URL}/v1/chat/sessions/${conversationId}/messages`, {
          content: textToSend,
          context_window: 5
        });

        if (response.data.error) {
          // Show notification about service unavailability
          setNotificationMessage('Chat service is temporarily unavailable. Using simulated responses.');
          setNotificationType('warning');
          setShowNotification(true);

          // Fallback to simulated response
          botMessage = {
            id: Date.now() + 1,
            text: `I received your message: "${textToSend}". This is a simulated response. Note: API service is temporarily unavailable.`,
            sender: 'bot',
            timestamp: new Date().toISOString(),
            isFallback: true
          };
        } else {
          botMessage = {
            id: Date.now() + 1,
            text: response.data.response || 'I received your message. This is a simulated response.',
            sender: 'bot',
            timestamp: new Date().toISOString()
          };
        }
      } catch (apiError) {
        console.error('API error:', apiError);

        // Show notification about API failure
        setNotificationMessage('Chat service is unavailable. Using simulated responses.');
        setNotificationType('warning');
        setShowNotification(true);

        // Fallback to simulated response if API fails
        botMessage = {
          id: Date.now() + 1,
          text: `I received your message: "${textToSend}". This is a simulated response. In a real implementation, this would come from the backend AI service. Note: API service is temporarily unavailable.`,
          sender: 'bot',
          timestamp: new Date().toISOString(),
          isFallback: true
        };
      }

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, there was an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date().toISOString(),
        isError: true
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSendSelectedText = async () => {
    const selectedText = getSelectedText();
    let textToSend = selectedText;

    if (!textToSend) {
      // Try to get selection details to provide better feedback
      const selectionDetails = getSelectionDetails();
      if (selectionDetails && selectionDetails.text) {
        textToSend = selectionDetails.text;
      }
    }

    if (textToSend) {
      // Add user message for selected text
      const userMessage = {
        id: Date.now(),
        text: textToSend,
        sender: 'user',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, userMessage]);
      setIsLoading(true);
      setError(null);

      try {
        // Try to send selected text to backend API using the ask-from-selection endpoint
        let botMessage;
        try {
          const response = await axios.post(`${API_BASE_URL}/v1/chat/ask-from-selection`, {
            content: textToSend,
            context_window: 5
          });

          if (response.data.error) {
            // Show notification about service unavailability
            setNotificationMessage('Chat service is temporarily unavailable. Using simulated responses.');
            setNotificationType('warning');
            setShowNotification(true);

            // Fallback to simulated response
            botMessage = {
              id: Date.now() + 1,
              text: `I received your selected text: "${textToSend}". This is a simulated response. Note: API service is temporarily unavailable.`,
              sender: 'bot',
              timestamp: new Date().toISOString(),
              isFallback: true
            };
          } else {
            botMessage = {
              id: Date.now() + 1,
              text: response.data.response || 'I received your selected text. This is a simulated response.',
              sender: 'bot',
              timestamp: new Date().toISOString()
            };
          }
        } catch (apiError) {
          console.error('API error for selected text:', apiError);

          // Show notification about API failure
          setNotificationMessage('Chat service is unavailable. Using simulated responses.');
          setNotificationType('warning');
          setShowNotification(true);

          // Fallback to simulated response if API fails
          botMessage = {
            id: Date.now() + 1,
            text: `I received your selected text: "${textToSend}". This is a simulated response. In a real implementation, this would come from the backend AI service. Note: API service is temporarily unavailable.`,
            sender: 'bot',
            timestamp: new Date().toISOString(),
            isFallback: true
          };
        }

        setMessages(prev => [...prev, botMessage]);
      } catch (error) {
        console.error('Error sending selected text:', error);
        const errorMessage = {
          id: Date.now() + 1,
          text: 'Sorry, there was an error processing your request. Please try again.',
          sender: 'bot',
          timestamp: new Date().toISOString(),
          isError: true
        };
        setMessages(prev => [...prev, errorMessage]);
      } finally {
        setIsLoading(false);
      }
    } else {
      alert('Please select some text on the page first.');
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const clearConversation = () => {
    try {
      setMessages([]);
      removeItem('chat_conversation');
      const newId = `session_${Date.now()}`;
      setConversationId(newId);
      setItem('chat_conversation', {
        conversationId: newId,
        messages: [],
        createdAt: new Date().toISOString()
      });
    } catch (storageError) {
      console.error('Error clearing conversation:', storageError);
      // Fallback: just clear the state
      setMessages([]);
      const newId = `session_${Date.now()}`;
      setConversationId(newId);

      // Show notification about localStorage issue
      setNotificationMessage('Failed to clear conversation from storage.');
      setNotificationType('warning');
      setShowNotification(true);
    }
  };

  const handleNotificationClose = () => {
    setShowNotification(false);
  };

  if (!isOpen) {
    return null; // Don't render anything if chat is not open
  }

  return (
    <div className="chat-widget">
      {showNotification && (
        <Notification
          message={notificationMessage}
          type={notificationType}
          onClose={handleNotificationClose}
        />
      )}
      <div className="chat-header">
        <h3>AI Assistant</h3>
        <div className="chat-controls">
          <button onClick={clearConversation} className="clear-conversation-btn" title="Clear Chat">
            🗑️
          </button>
          <button onClick={onClose} className="close-chat-btn" title="Close Chat">
            ✕
          </button>
        </div>
      </div>

      <div className="chat-messages">
        {error && (
          <div className="chat-error">
            {error}
          </div>
        )}
        {messages.length === 0 ? (
          <div className="chat-welcome">
            <p>Hello! I'm your AI assistant. How can I help you with the Humanoid Robotics content today?</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`chat-message ${message.sender === 'user' ? 'user' : 'bot'} ${message.isError ? 'error-message' : ''} ${message.isFallback ? 'fallback-message' : ''}`}
            >
              <div className="message-content">
                {message.text}
              </div>
              <div className="message-timestamp">
                {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className="chat-input-form">
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Type your message here..."
          className="chat-input"
          rows="3"
        />
        <div className="chat-input-buttons">
          <button
            onClick={handleSendMessage}
            disabled={!inputValue.trim() || isLoading}
            className="send-button"
          >
            Send
          </button>
          <button
            onClick={handleSendSelectedText}
            disabled={isLoading}
            className="send-selected-button"
          >
            Send Selected Text
          </button>
        </div>
      </div>
    </div>
  );
};

export default ChatWidget;