/**
 * Utility functions for localStorage operations with error handling
 */

// Maximum localStorage size limit (5MB as per specification)
const MAX_STORAGE_SIZE = 5 * 1024 * 1024; // 5MB in bytes

/**
 * Get item from localStorage
 * @param {string} key - The key to retrieve
 * @returns {any} Parsed value or null if not found or error
 */
export const getItem = (key) => {
  try {
    // Check if localStorage is available
    if (!isStorageAvailable()) {
      console.warn(`localStorage not available, cannot get item with key "${key}"`);
      return null;
    }

    const item = localStorage.getItem(key);
    return item ? JSON.parse(item) : null;
  } catch (error) {
    console.error(`Error getting item from localStorage with key "${key}":`, error);
    // Try to return a fallback value
    try {
      // In case of parse error, return the raw value
      const rawItem = localStorage.getItem(key);
      return rawItem;
    } catch {
      return null;
    }
  }
};

/**
 * Set item in localStorage
 * @param {string} key - The key to set
 * @param {any} value - The value to store
 * @returns {boolean} True if successful, false otherwise
 */
export const setItem = (key, value) => {
  try {
    // Check if localStorage is available
    if (!isStorageAvailable()) {
      console.warn(`localStorage not available, cannot set item with key "${key}"`);
      return false;
    }

    // Check if we're approaching storage limits
    let serializedValue;
    try {
      serializedValue = JSON.stringify(value);
    } catch (serializeError) {
      console.error(`Error serializing value for key "${key}":`, serializeError);
      return false;
    }

    const usedSize = new Blob([serializedValue]).size;
    const currentUsage = JSON.stringify(localStorage).length;

    // Check if adding this item would exceed limits
    if (currentUsage + usedSize > MAX_STORAGE_SIZE * 0.9) { // Use 90% as safety margin
      console.warn('Approaching localStorage limit, attempting cleanup');
      cleanupOldEntries();
    }

    localStorage.setItem(key, serializedValue);
    return true;
  } catch (error) {
    console.error(`Error setting item in localStorage with key "${key}":`, error);

    if (error.name === 'QuotaExceededError') {
      console.warn('localStorage quota exceeded, attempting cleanup');
      cleanupOldEntries();
      // Try again after cleanup
      try {
        const serializedValue = JSON.stringify(value);
        localStorage.setItem(key, serializedValue);
        return true;
      } catch (retryError) {
        console.error('Failed to set item even after cleanup:', retryError);
      }
    } else if (error.name === 'SecurityError') {
      console.error('Security error accessing localStorage (possibly in private browsing mode)');
    }
    return false;
  }
};

/**
 * Remove item from localStorage
 * @param {string} key - The key to remove
 * @returns {boolean} True if successful, false otherwise
 */
export const removeItem = (key) => {
  try {
    localStorage.removeItem(key);
    return true;
  } catch (error) {
    console.error(`Error removing item from localStorage with key "${key}":`, error);
    return false;
  }
};

/**
 * Clear all items from localStorage
 * @returns {boolean} True if successful, false otherwise
 */
export const clear = () => {
  try {
    localStorage.clear();
    return true;
  } catch (error) {
    console.error('Error clearing localStorage:', error);
    return false;
  }
};

/**
 * Get current storage usage information
 * @returns {Object} Object with usage information
 */
export const getStorageInfo = () => {
  try {
    const allItems = JSON.stringify(localStorage);
    const usedBytes = new Blob([allItems]).size;
    const usedPercentage = (usedBytes / MAX_STORAGE_SIZE) * 100;

    return {
      usedBytes,
      usedPercentage,
      maxBytes: MAX_STORAGE_SIZE,
      itemCount: localStorage.length
    };
  } catch (error) {
    console.error('Error getting storage info:', error);
    return null;
  }
};

/**
 * Check if we're approaching storage limits
 * @returns {boolean} True if approaching limit, false otherwise
 */
export const isApproachingLimit = () => {
  try {
    const info = getStorageInfo();
    if (!info) return false;

    // Return true if we're using more than 80% of available space
    return info.usedPercentage > 80;
  } catch (error) {
    console.error('Error checking storage limit:', error);
    return false;
  }
};

/**
 * Get all keys that start with a specific prefix
 * @param {string} prefix - The prefix to search for
 * @returns {Array} Array of keys that match the prefix
 */
export const getKeysByPrefix = (prefix) => {
  try {
    const keys = [];
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key && key.startsWith(prefix)) {
        keys.push(key);
      }
    }
    return keys;
  } catch (error) {
    console.error('Error getting keys by prefix:', error);
    return [];
  }
};

/**
 * Remove items by prefix to free up space
 * @param {string} prefix - The prefix of keys to remove
 * @returns {number} Number of items removed
 */
export const removeItemsByPrefix = (prefix) => {
  try {
    const keys = getKeysByPrefix(prefix);
    let removedCount = 0;

    keys.forEach(key => {
      try {
        localStorage.removeItem(key);
        removedCount++;
      } catch (removeError) {
        console.error(`Error removing item with key "${key}":`, removeError);
      }
    });

    return removedCount;
  } catch (error) {
    console.error('Error removing items by prefix:', error);
    return 0;
  }
};

/**
 * Cleanup old entries to free up space
 * @private
 */
const cleanupOldEntries = () => {
  try {
    const keys = Object.keys(localStorage);
    // Sort keys by modification time if available (using a custom timestamp)
    // For now, we'll just remove the oldest entries based on when they were added
    // This is a simplified approach - in a real implementation, we'd track timestamps

    // Remove oldest entries until we're under the limit
    while (localStorage.length > 0) {
      const firstKey = Object.keys(localStorage)[0];
      if (firstKey) {
        localStorage.removeItem(firstKey);
      } else {
        break;
      }

      // Check if we're under the limit now
      const currentUsage = JSON.stringify(localStorage).length;
      if (currentUsage < MAX_STORAGE_SIZE * 0.8) { // Target 80% usage
        break;
      }
    }
  } catch (error) {
    console.error('Error during localStorage cleanup:', error);
  }
};

/**
 * Check if localStorage is available and working
 * @returns {boolean} True if available, false otherwise
 */
export const isStorageAvailable = () => {
  try {
    const testKey = '__storage_test__';
    localStorage.setItem(testKey, testKey);
    localStorage.removeItem(testKey);
    return true;
  } catch (error) {
    console.error('localStorage not available:', error);
    return false;
  }
};

// Export default object with all functions
const LocalStorageService = {
  getItem,
  setItem,
  removeItem,
  clear,
  getStorageInfo,
  isStorageAvailable
};

export default LocalStorageService;