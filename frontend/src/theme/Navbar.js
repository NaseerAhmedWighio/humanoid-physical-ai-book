import React, { useState, useEffect } from 'react';
import Navbar from '@theme-original/Navbar';
import SearchModal from '@site/src/components/SearchModal';

export default function NavbarWrapper(props) {
  const [isSearchOpen, setIsSearchOpen] = useState(false);

  useEffect(() => {
    const handleKeyDown = (e) => {
      // Check for Ctrl+K or Cmd+K
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        setIsSearchOpen(prev => !prev);
      }
      // Check for forward slash as alternative
      if (e.key === '/' && !e.ctrlKey && !e.metaKey && e.target.tagName !== 'INPUT') {
        e.preventDefault();
        setIsSearchOpen(true);
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, []);

  // Custom search handler for the visible search button
  const handleSearchToggle = () => {
    setIsSearchOpen(prev => !prev);
  };

  // Add a global function to handle search toggle from the search button
  useEffect(() => {
    window.handleSearchToggle = handleSearchToggle;
    return () => {
      delete window.handleSearchToggle;
    };
  }, []);

  return (
    <>
      <Navbar {...props} />
      <SearchModal
        isOpen={isSearchOpen}
        onClose={() => setIsSearchOpen(false)}
      />
    </>
  );
}