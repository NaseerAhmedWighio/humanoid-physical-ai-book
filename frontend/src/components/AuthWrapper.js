import React from 'react';
import { BetterAuthProvider } from '../context/BetterAuthContext';

const AuthWrapper = ({ children }) => {
  return (
    <BetterAuthProvider>
      {children}
    </BetterAuthProvider>
  );
};

export default AuthWrapper;