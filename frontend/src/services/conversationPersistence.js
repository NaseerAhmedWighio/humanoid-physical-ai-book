import { getItem, setItem, removeItem, getStorageInfo } from './localStorage';

// Constants for conversation persistence
const CONVERSATION_KEY = 'chat_conversation';
const CONVERSATION_HISTORY_KEY = 'chat_conversation_history';
const MAX_CONVERSATIONS = 10; // Keep only the last 10 conversations
const MAX_MESSAGE_HISTORY = 50; // Keep only the last 50 messages per conversation

/**
 * Save a conversation to localStorage
 * @param {Object} conversation - The conversation object to save
 * @returns {boolean} - True if successful, false otherwise
 */
export const saveConversation = (conversation) => {
  try {
    // Validate conversation object
    if (!conversation || !conversation.conversationId) {
      console.error('Invalid conversation object provided to saveConversation');
      return false;
    }

    // Limit message history to prevent storage overflow
    const limitedConversation = {
      ...conversation,
      messages: conversation.messages ? conversation.messages.slice(-MAX_MESSAGE_HISTORY) : []
    };

    return setItem(CONVERSATION_KEY, limitedConversation);
  } catch (error) {
    console.error('Error saving conversation:', error);
    return false;
  }
};

/**
 * Load the current conversation from localStorage
 * @returns {Object|null} - The conversation object or null if not found
 */
export const loadConversation = () => {
  try {
    return getItem(CONVERSATION_KEY);
  } catch (error) {
    console.error('Error loading conversation:', error);
    return null;
  }
};

/**
 * Clear the current conversation from localStorage
 * @returns {boolean} - True if successful, false otherwise
 */
export const clearConversation = () => {
  try {
    return removeItem(CONVERSATION_KEY);
  } catch (error) {
    console.error('Error clearing conversation:', error);
    return false;
  }
};

/**
 * Save conversation to history
 * @param {Object} conversation - The conversation to save to history
 * @returns {boolean} - True if successful, false otherwise
 */
export const saveConversationToHistory = (conversation) => {
  try {
    // Validate conversation object
    if (!conversation || !conversation.conversationId) {
      console.error('Invalid conversation object provided to saveConversationToHistory');
      return false;
    }

    // Get existing history
    const history = getItem(CONVERSATION_HISTORY_KEY) || [];

    // Check if conversation already exists in history
    const existingIndex = history.findIndex(conv => conv.conversationId === conversation.conversationId);

    if (existingIndex !== -1) {
      // Update existing conversation
      history[existingIndex] = {
        ...conversation,
        lastUpdated: new Date().toISOString()
      };
    } else {
      // Add new conversation to history
      const conversationForHistory = {
        ...conversation,
        lastUpdated: new Date().toISOString(),
        preview: conversation.messages && conversation.messages.length > 0
          ? conversation.messages[conversation.messages.length - 1].text.substring(0, 50) + '...'
          : 'New conversation'
      };
      history.unshift(conversationForHistory);
    }

    // Limit history to MAX_CONVERSATIONS
    if (history.length > MAX_CONVERSATIONS) {
      history.splice(MAX_CONVERSATIONS);
    }

    return setItem(CONVERSATION_HISTORY_KEY, history);
  } catch (error) {
    console.error('Error saving conversation to history:', error);
    return false;
  }
};

/**
 * Load conversation history
 * @returns {Array} - Array of conversation objects
 */
export const loadConversationHistory = () => {
  try {
    return getItem(CONVERSATION_HISTORY_KEY) || [];
  } catch (error) {
    console.error('Error loading conversation history:', error);
    return [];
  }
};

/**
 * Delete a conversation from history
 * @param {string} conversationId - The ID of the conversation to delete
 * @returns {boolean} - True if successful, false otherwise
 */
export const deleteConversationFromHistory = (conversationId) => {
  try {
    const history = getItem(CONVERSATION_HISTORY_KEY) || [];
    const filteredHistory = history.filter(conv => conv.conversationId !== conversationId);
    return setItem(CONVERSATION_HISTORY_KEY, filteredHistory);
  } catch (error) {
    console.error('Error deleting conversation from history:', error);
    return false;
  }
};

/**
 * Clear all conversation history
 * @returns {boolean} - True if successful, false otherwise
 */
export const clearConversationHistory = () => {
  try {
    return removeItem(CONVERSATION_HISTORY_KEY);
  } catch (error) {
    console.error('Error clearing conversation history:', error);
    return false;
  }
};

/**
 * Check if we're approaching storage limits and manage accordingly
 * @returns {boolean} - True if storage is sufficient, false if action was needed
 */
export const checkAndManageStorage = () => {
  try {
    const storageInfo = getStorageInfo();
    if (!storageInfo) {
      return false;
    }

    // If we're using more than 80% of available storage, clean up old conversations
    if (storageInfo.usedPercentage > 80) {
      console.warn('Storage usage is high, cleaning up old conversations');
      cleanupOldConversations();
      return false;
    }

    return true;
  } catch (error) {
    console.error('Error checking storage:', error);
    return false;
  }
};

/**
 * Clean up old conversations to free up space
 * @private
 */
const cleanupOldConversations = () => {
  try {
    // Clean up conversation history
    const history = getItem(CONVERSATION_HISTORY_KEY) || [];

    // Keep only the most recent conversations
    if (history.length > MAX_CONVERSATIONS / 2) { // Keep only half when storage is tight
      const recentHistory = history.slice(0, Math.floor(MAX_CONVERSATIONS / 2));
      setItem(CONVERSATION_HISTORY_KEY, recentHistory);
    }
  } catch (error) {
    console.error('Error during conversation cleanup:', error);
  }
};

/**
 * Export all functions as a service object
 */
const ConversationPersistenceService = {
  saveConversation,
  loadConversation,
  clearConversation,
  saveConversationToHistory,
  loadConversationHistory,
  deleteConversationFromHistory,
  clearConversationHistory,
  checkAndManageStorage
};

export default ConversationPersistenceService;