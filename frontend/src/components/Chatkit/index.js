import React, { useState, useEffect, useRef } from 'react';
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message,
  MessageInput,
  MessageSeparator,
  TypingIndicator
} from '@chatscope/chat-ui-kit-react';
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';
import { getItem, setItem } from '../../services/localStorage';
import { useUserPreference } from '../../context/UserPreferenceContext';
import axios from 'axios';
import { API_BASE_URL } from '../../constants/api';

const Chatkit = ({ isOpen = false, onClose, initialMessage = null, onInitialMessageProcessed }) => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionData, setSessionData] = useState(null);
  const { personalizationEnabled, hardwarePreference } = useUserPreference();
  const messagesEndRef = useRef(null);

  // Load conversation from localStorage on component mount
  useEffect(() => {
    if (isOpen) {
      const savedConversation = getItem('chatkit_conversation');
      if (savedConversation) {
        setMessages(savedConversation.messages || []);
        setSessionData(savedConversation.sessionData || null);
      } else {
        // Initialize with a new session ID
        setSessionData({ session_id: `session_${Date.now()}` });
      }

      // Handle initial message if provided
      if (initialMessage) {
        handleSendMessage(initialMessage);
        if (onInitialMessageProcessed) {
          onInitialMessageProcessed();
        }
      }
    }
  }, [isOpen, initialMessage, onInitialMessageProcessed]);

  // Save conversation to localStorage whenever messages change
  useEffect(() => {
    if (sessionData || messages.length > 0) {
      const conversationData = {
        messages,
        sessionData: sessionData || null,
        updatedAt: new Date().toISOString()
      };
      setItem('chatkit_conversation', conversationData);
    }
  }, [messages, sessionData]);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };


  const handleSendMessage = async (messageText) => {
    if (!messageText.trim() || isLoading) return;

    // Add user message to UI immediately
    const userMessage = {
      id: Date.now(),
      message: messageText,
      sender: 'You',
      direction: 'outgoing',
      position: 'normal',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Send message to backend RAG agent using the sessions API
      const response = await axios.post(
        `${API_BASE_URL}/v1/chat/sessions/${sessionData?.session_id || 'temp'}/messages`,
        {
          content: messageText,
          context_window: 5,
          personalization_enabled: personalizationEnabled,
          hardware_preference: hardwarePreference
        }
      );

      // Add bot response to messages
      const botMessage = {
        id: Date.now() + 1,
        message: response.data.response || 'I received your message. This is a simulated response.',
        sender: 'AI Assistant',
        direction: 'incoming',
        position: 'normal',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message
      const errorMessage = {
        id: Date.now() + 1,
        message: 'Sorry, there was an error processing your request. Please try again.',
        sender: 'AI Assistant',
        direction: 'incoming',
        position: 'normal',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div className="chatkit-container" style={{
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      width: '400px',
      height: '600px',
      zIndex: 1000,
      borderRadius: '12px',
      boxShadow: '0 10px 30px rgba(0, 0, 0, 0.3)',
      backgroundColor: 'rgba(26, 26, 46, 0.9)', // Match website dark theme
      border: '1px solid rgba(0, 238, 255, 0.3)', // Match website primary color
      backdropFilter: 'blur(10px)',
      color: '#ffffff', // White text to match website
      display: 'flex',
      flexDirection: 'column'
    }}>
      {/* Chat Header with Close Button */}
      <div style={{
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        padding: '12px 16px',
        backgroundColor: 'rgba(10, 10, 10, 0.7)',
        borderTopLeftRadius: '12px',
        borderTopRightRadius: '12px',
        borderBottom: '1px solid rgba(0, 238, 255, 0.3)',
        color: '#00eeff'
      }}>
        <h3 style={{ margin: 0, fontSize: '16px', fontWeight: '600' }}>AI Assistant</h3>
        <button
          onClick={onClose}
          style={{
            background: 'none',
            border: 'none',
            color: '#00eeff',
            fontSize: '20px',
            cursor: 'pointer',
            padding: '4px',
            borderRadius: '4px',
            width: '30px',
            height: '30px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center'
          }}
          aria-label="Close chat"
        >
          ×
        </button>
      </div>

      <MainContainer style={{
        height: 'calc(100% - 50px)', // Account for header height
        backgroundColor: 'rgba(26, 26, 46, 0.9)',
        borderBottomLeftRadius: '12px',
        borderBottomRightRadius: '12px',
        border: 'none'
      }}>
        <ChatContainer style={{ backgroundColor: 'rgba(26, 26, 46, 0.9)' }}>
          <MessageList
            scrollBehavior="smooth"
            style={{
              backgroundColor: 'rgba(26, 26, 46, 0.7)',
              padding: '10px'
            }}
            typingIndicator={isLoading ? <TypingIndicator
              content="AI Assistant is typing"
              style={{ color: '#00eeff' }}
            /> : null}
          >
            {messages.length === 0 ? (
              <div style={{
                display: 'flex',
                justifyContent: 'center',
                alignItems: 'center',
                height: '100%',
                padding: '20px',
                textAlign: 'center',
                color: '#b0b0ff', // Match website secondary text color
                backgroundColor: 'rgba(26, 26, 46, 0.7)'
              }}>
                <p>Hello! I'm your AI assistant for Humanoid Robotics. How can I help you today?</p>
              </div>
            ) : (
              messages.map((msg) => (
                <Message
                  key={msg.id}
                  style={{
                    backgroundColor: msg.direction === 'incoming'
                      ? 'rgba(0, 238, 255, 0.1)' // Subtle cyan for bot messages
                      : 'rgba(0, 119, 255, 0.2)', // Subtle blue for user messages
                    borderRadius: '8px',
                    padding: '8px',
                    margin: '5px 0'
                  }}
                  model={{
                    message: msg.message,
                    sender: msg.sender,
                    direction: msg.direction,
                    position: msg.position
                  }}
                />
              ))
            )}
            <div ref={messagesEndRef} />
          </MessageList>

          <MessageInput
            placeholder="Type your message here..."
            onSend={handleSendMessage}
            disabled={isLoading}
            style={{
              backgroundColor: 'rgba(10, 10, 10, 0.7)',
              border: '1px solid rgba(0, 238, 255, 0.3)',
              borderRadius: '8px',
              color: '#ffffff',
              padding: '12px'
            }}
            sendButton={{
              backgroundColor: '#00eeff',
              color: '#0a0a0a',
              borderRadius: '4px',
              padding: '8px 16px',
              margin: '4px',
              '&:hover': {
                backgroundColor: '#17f0ff'
              }
            }}
          />
        </ChatContainer>
      </MainContainer>
    </div>
  );
};

export default Chatkit;