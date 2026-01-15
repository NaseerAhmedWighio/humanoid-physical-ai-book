import React, { useEffect, useState } from "react";
import Link from "@docusaurus/Link";
import { useBetterAuth } from "@site/src/context/BetterAuthContext";
import './NavbarItemCustomAuthButtons.css';

function NavbarItemCustomAuthButtons() {
  const { user, logout, loading } = useBetterAuth();

  const [loggedIn, setLoggedIn] = useState(false);

  useEffect(() => {
    // run only in browser
    if (typeof window !== "undefined") {
      const token = localStorage.getItem("better_auth_token");
      setLoggedIn(!!token);
    }
  }, []);

  const handleLogout = () => {
    logout();
    localStorage.removeItem("better_auth_token");
    setLoggedIn(false);
  };

  if (loading) {
    return (
      <div className="navbar__item">
        <span className="navbar__link">Loading...</span>
      </div>
    );
  }

  // 🔥 Priority: if we have token OR user => logged in
  if (loggedIn || user) {
    const userName = user?.email
      ? user.email.split("@")[0]
      : "User";

    const userInitials = userName.charAt(0).toUpperCase();

    return (
      <div className="navbar__item ">
        <div className="dropdown dropdown--hoverable dropdown--right">
          <span className="navbar__link navbar__link--subtle">
            <div className="user-profile-container">
              {/* <div className="user-avatar">
                {userInitials}
              </div> */}
              <span className="user-name">{userName}</span>
              {/* <span className="dropdown-arrow">▾</span> */}
            </div>
          </span>

          <ul className="dropdown__menu">
            <li>
              <button
                className="dropdown__link"
                onClick={handleLogout}
                style={{
                  background: "none",
                  border: "none",
                  width: "100%",
                  textAlign: "left",
                  cursor: "pointer",
                }}
              >
                Logout
              </button>
            </li>
          </ul>
        </div>
      </div>
    );
  }

  // 🚪 Not logged in
  return (
    <div className="navbar__item">
      <div className="navbar__auth-buttons">
        <Link
          className="button button--sm button--outline button--primary margin-right--sm"
          to="/signin"
        >
          Sign In
        </Link>

        <Link
          className="button button--sm button--primary"
          to="/signup"
        >
          Sign Up
        </Link>
      </div>
    </div>
  );
}

export default NavbarItemCustomAuthButtons;
