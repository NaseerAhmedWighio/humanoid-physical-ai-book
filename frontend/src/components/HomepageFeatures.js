import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: '🤖 Module 1: Introduction to ROS 2',
    description: (
      <>
        Understanding the Robot Operating System 2 framework, node-based architecture,
        communication patterns, and message passing. Learn practical implementation
        of ROS 2 concepts for humanoid robotics applications.
      </>
    ),
    weeks: 'Weeks 1-3',
    link: '/docs/module-1-ros2'
  },
  {
    title: '🎮 Module 2: Simulation with Gazebo and Unity',
    description: (
      <>
        Advanced simulation environments for robotics, Gazebo physics simulation,
        sensor modeling, Unity integration for high-fidelity graphics,
        and simulation-to-reality transfer techniques.
      </>
    ),
    weeks: 'Weeks 4-6',
    link: '/docs/module-2-simulation'
  },
  {
    title: '🚀 Module 3: NVIDIA Isaac for Humanoid Control',
    description: (
      <>
        GPU-accelerated robotics with NVIDIA Isaac, perception systems and control algorithms,
        Isaac Sim for AI training environments, and integration with real hardware platforms.
      </>
    ),
    weeks: 'Weeks 7-9',
    link: '/docs/module-3-nvidia-isaac'
  },
  {
    title: '🧠 Module 4: Vision Language Action (VLA) Models',
    description: (
      <>
        Advanced AI integration for robotic systems, vision-language-action models for natural
        interaction, deep learning for perception and control, and real-world deployment strategies.
      </>
    ),
    weeks: 'Weeks 10-13',
    link: '/docs/module-4-vla'
  },
];

function Feature({title, description, weeks, link}) {
  return (
    <div className="module-card-wrapper">
      <div className="module-card">
        <div className="text--center">
          <div className={styles.featureSvg}>
            <h3>{title}</h3>
            <p className={styles.weeks}>{weeks}</p>
          </div>
        </div>
        <div className="text--center padding-horiz--md">
          <p>{description}</p>
          <a href={link} className="button button--secondary button--sm">
            Learn More
          </a>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  const [activeTab, setActiveTab] = useState('week-1');
  const [isVisible, setIsVisible] = useState({});

  const weeksData = {
    'week-1': {
      title: 'Week 1-3: ROS 2 Fundamentals',
      content: 'Introduction to ROS 2 architecture, nodes, topics, services, and actions. Understanding the communication layer and developing basic ROS 2 packages for humanoid robots.'
    },
    'week-4': {
      title: 'Week 4-6: Simulation Environments',
      content: 'Working with Gazebo and Unity simulation environments. Creating realistic physics models, sensor simulation, and testing humanoid robot behaviors in virtual worlds.'
    },
    'week-7': {
      title: 'Week 7-9: NVIDIA Isaac Integration',
      content: 'Using NVIDIA Isaac for advanced humanoid control. GPU-accelerated perception, Isaac Sim for training environments, and hardware integration techniques.'
    },
    'week-10': {
      title: 'Week 10-13: AI Integration',
      content: 'Vision Language Action models for humanoid robots. Advanced AI techniques for perception, decision making, and natural human-robot interaction.'
    }
  };

  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            setIsVisible(prev => ({
              ...prev,
              [entry.target.id]: true
            }));
          }
        });
      },
      { threshold: 0.1 }
    );

    const elements = document.querySelectorAll('[data-animate]');
    elements.forEach((el) => observer.observe(el));

    return () => observer.disconnect();
  }, []);

  return (
    <section className={styles.features}>
      <div className="container">
        {/* Modules Section with Responsive Grid */}
        <div className="">
          <div className="container">
            <h2 className={styles.sectionTitle}>Course Modules</h2>
            <p className={styles.sectionSubtitle}>Explore our comprehensive 4-module curriculum</p>
            <div className="modules-grid-responsive">
              {FeatureList.map((props, idx) => (
                <Feature key={idx} {...props} />
              ))}
            </div>
          </div>
        </div>

        {/* Weeks Tabs Section */}
        <div className="">
          <div className="container">
            <h2 className={styles.sectionTitle}>Course Timeline</h2>
            <p className={styles.sectionSubtitle}>Navigate through the 13-week course structure</p>

            <div className="tabs-container">
              <div className="weeks-tabs">
                <a
                  href="/docs/weeks/week-01-02"
                  className={`week-tab ${activeTab === 'week-1' ? 'active' : ''}`}
                  onClick={(e) => {
                    e.preventDefault();
                    setActiveTab('week-1');
                    window.location.href = '/docs/weeks/week-01-02';
                  }}
                >
                  Weeks 1-3
                </a>
                <a
                  href="/docs/weeks/week-03"
                  className={`week-tab ${activeTab === 'week-4' ? 'active' : ''}`}
                  onClick={(e) => {
                    e.preventDefault();
                    setActiveTab('week-4');
                    window.location.href = '/docs/weeks/week-03';
                  }}
                >
                  Weeks 4-6
                </a>
                <a
                  href="/docs/weeks/week-04-05"
                  className={`week-tab ${activeTab === 'week-7' ? 'active' : ''}`}
                  onClick={(e) => {
                    e.preventDefault();
                    setActiveTab('week-7');
                    window.location.href = '/docs/weeks/week-04-05';
                  }}
                >
                  Weeks 7-9
                </a>
                <a
                  href="/docs/weeks/week-10-13"
                  className={`week-tab ${activeTab === 'week-10' ? 'active' : ''}`}
                  onClick={(e) => {
                    e.preventDefault();
                    setActiveTab('week-10');
                    window.location.href = '/docs/weeks/week-10-13';
                  }}
                >
                  Weeks 10-13
                </a>
              </div>

              <div className="week-content">
                <h3>{weeksData[activeTab].title}</h3>
                <p>{weeksData[activeTab].content}</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}