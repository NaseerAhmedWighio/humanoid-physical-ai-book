import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import './index.css';

// Import a default image or create a placeholder div instead

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <>
      <header className="hero hero--primary heroBanner">
        <div className="container">
          <div className="hero-content">
            <div className="hero-text">
              <h1 className="hero__title">{siteConfig.title}</h1>
              <p className="hero__subtitle">Welcome to the Future of Human-Robot Interaction</p>
              <p className="hero-description">
                The comprehensive textbook on Physical AI & Humanoid Robotics. This 13-week course covers
                fundamental concepts, practical implementations, and ethical considerations of creating
                intelligent humanoid robots that can interact with human environments.
              </p>
              <div className="hero-buttons">
                <Link
                  className="button button--secondary button--lg"
                  to="/docs/intro">
                  Get Started
                </Link>
                <Link
                  className="button button--outline button--lg"
                  to="/docs/module-1-ros2">
                  Explore Modules
                </Link>
              </div>
            </div>
            <div className="hero-image-section">
              <div className="hero-image-container">
                <img
                  src="img/banner.jpg"
                  alt="Humanoid Robot Visualization"
                  className="hero-image"
                />
              </div>
            </div>
          </div>

          {/* Tags Section */}
          <div className="hero-tags">
            <span className="tag">ROS 2</span>
            <span className="tag">Humanoid Robotics</span>
            <span className="tag">Physical AI</span>
            <span className="tag">Simulation</span>
            <span className="tag">NVIDIA Isaac</span>
            <span className="tag">Vision Language Action</span>
            <span className="tag">Machine Learning</span>
            <span className="tag">Human-Robot Interaction</span>
          </div>
        </div>
      </header>
      {/* Tabs Section */}
      <section className="tabs-section">
        <div className="container">
          <div className="tabs-container">
            <div className="tabs-header">
              <h2>Course Modules</h2>
              <p>Explore our comprehensive curriculum designed for humanoid robotics</p>
            </div>
            <div className="tabs-navigation">
              <button className="tab-button active">Module 1</button>
              <button className="tab-button">Module 2</button>
              <button className="tab-button">Module 3</button>
              <button className="tab-button">Module 4</button>
            </div>
            <div className="tab-content">
              <p>Learn the fundamentals of ROS 2 and its application in humanoid robotics.</p>
            </div>
          </div>
        </div>
      </section>
    </>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook - Comprehensive 13-week course">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}