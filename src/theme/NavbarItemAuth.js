import React from "react";

export default function NavbarItemAuth(props) {
  // Minimal implementation to avoid crashes - will implement full auth state later
  return (
    <div className="navbar__item">
      <div className="button-group button-group--spacing-md">
        <a href="/login" className="button button--secondary button--sm">
          Login
        </a>
        <a href="/signup" className="button button--primary button--sm">
          Sign Up
        </a>
      </div>
    </div>
  );
}