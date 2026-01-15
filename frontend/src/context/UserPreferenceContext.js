import React, { createContext, useContext, useReducer, useEffect } from 'react';
import { getItem, setItem } from '../services/localStorage';

const UserPreferenceContext = createContext();

// Initial state for user preferences
const initialState = {
  hardwarePreference: 'laptop', // Default to laptop
  personalizationEnabled: false,
  loading: false,
  error: null
};

// Action types
const actionTypes = {
  SET_HARDWARE_PREFERENCE: 'SET_HARDWARE_PREFERENCE',
  TOGGLE_PERSONALIZATION: 'TOGGLE_PERSONALIZATION',
  SET_LOADING: 'SET_LOADING',
  SET_ERROR: 'SET_ERROR',
  RESET_TO_DEFAULT: 'RESET_TO_DEFAULT'
};

// Reducer function
const preferenceReducer = (state, action) => {
  switch (action.type) {
    case actionTypes.SET_HARDWARE_PREFERENCE:
      return {
        ...state,
        hardwarePreference: action.payload,
        loading: false,
        error: null
      };
    case actionTypes.TOGGLE_PERSONALIZATION:
      return {
        ...state,
        personalizationEnabled: action.payload,
        loading: false,
        error: null
      };
    case actionTypes.SET_LOADING:
      return {
        ...state,
        loading: action.payload
      };
    case actionTypes.SET_ERROR:
      return {
        ...state,
        error: action.payload,
        loading: false
      };
    case actionTypes.RESET_TO_DEFAULT:
      return {
        ...initialState,
        loading: false,
        error: null
      };
    default:
      return state;
  }
};

// Provider component
export const UserPreferenceProvider = ({ children }) => {
  const [state, dispatch] = useReducer(preferenceReducer, initialState);

  // Load preferences from localStorage on initial render
  useEffect(() => {
    const savedPreferences = getItem('userPreferences');
    if (savedPreferences) {
      dispatch({ type: actionTypes.SET_HARDWARE_PREFERENCE, payload: savedPreferences.hardwarePreference });
      dispatch({ type: actionTypes.TOGGLE_PERSONALIZATION, payload: savedPreferences.personalizationEnabled });
    }
  }, []);

  // Save preferences to localStorage whenever they change
  useEffect(() => {
    const preferencesToSave = {
      hardwarePreference: state.hardwarePreference,
      personalizationEnabled: state.personalizationEnabled
    };
    setItem('userPreferences', preferencesToSave);
  }, [state.hardwarePreference, state.personalizationEnabled]);

  // Actions
  const setHardwarePreference = (preference) => {
    if (!['mobile', 'laptop', 'physical_robot'].includes(preference)) {
      dispatch({ type: actionTypes.SET_ERROR, payload: 'Invalid hardware preference' });
      return;
    }
    dispatch({ type: actionTypes.SET_HARDWARE_PREFERENCE, payload: preference });
  };

  const togglePersonalization = () => {
    dispatch({ type: actionTypes.TOGGLE_PERSONALIZATION, payload: !state.personalizationEnabled });
  };

  const enablePersonalization = () => {
    dispatch({ type: actionTypes.TOGGLE_PERSONALIZATION, payload: true });
  };

  const disablePersonalization = () => {
    dispatch({ type: actionTypes.TOGGLE_PERSONALIZATION, payload: false });
  };

  const resetToDefault = () => {
    dispatch({ type: actionTypes.RESET_TO_DEFAULT });
  };

  const value = {
    ...state,
    setHardwarePreference,
    togglePersonalization,
    enablePersonalization,
    disablePersonalization,
    resetToDefault
  };

  return (
    <UserPreferenceContext.Provider value={value}>
      {children}
    </UserPreferenceContext.Provider>
  );
};

// Custom hook to use the context
export const useUserPreference = () => {
  const context = useContext(UserPreferenceContext);
  if (!context) {
    throw new Error('useUserPreference must be used within a UserPreferenceProvider');
  }
  return context;
};

export default UserPreferenceContext;