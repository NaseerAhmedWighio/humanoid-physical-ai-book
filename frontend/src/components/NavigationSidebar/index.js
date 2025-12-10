import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

function NavigationSidebar({ currentPath = '' }) {
  const [courseStructure, setCourseStructure] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // In a real implementation, this would fetch from the backend API
    // For now, we'll use mock data representing the 13-week course structure
    const mockCourseStructure = [
      {
        id: 'module-1',
        title: 'Module 1: Introduction to ROS 2',
        type: 'module',
        items: [
          { id: 'week-1', title: 'Week 1: ROS 2 Architecture Overview', type: 'week', path: '/docs/weeks/week-1' },
          { id: 'week-2', title: 'Week 2: Nodes, Topics, Services', type: 'week', path: '/docs/weeks/week-2' },
          { id: 'week-3', title: 'Week 3: Actions and Parameters', type: 'week', path: '/docs/weeks/week-3' }
        ]
      },
      {
        id: 'module-2',
        title: 'Module 2: Simulation with Gazebo and Unity',
        type: 'module',
        items: [
          { id: 'week-4', title: 'Week 4: Gazebo Fundamentals', type: 'week', path: '/docs/weeks/week-4' },
          { id: 'week-5', title: 'Week 5: Unity Robotics Setup', type: 'week', path: '/docs/weeks/week-5' },
          { id: 'week-6', title: 'Week 6: Physics-Based Simulation', type: 'week', path: '/docs/weeks/week-6' }
        ]
      },
      {
        id: 'module-3',
        title: 'Module 3: NVIDIA Isaac for Humanoid Control',
        type: 'module',
        items: [
          { id: 'week-7', title: 'Week 7: Isaac Sim Environment', type: 'week', path: '/docs/weeks/week-7' },
          { id: 'week-8', title: 'Week 8: Perception Systems', type: 'week', path: '/docs/weeks/week-8' },
          { id: 'week-9', title: 'Week 9: Control Algorithms', type: 'week', path: '/docs/weeks/week-9' }
        ]
      },
      {
        id: 'module-4',
        title: 'Module 4: Vision Language Action (VLA) Models',
        type: 'module',
        items: [
          { id: 'week-10', title: 'Week 10: VLA Architecture', type: 'week', path: '/docs/weeks/week-10' },
          { id: 'week-11', title: 'Week 11: Vision-Language Integration', type: 'week', path: '/docs/weeks/week-11' },
          { id: 'week-12', title: 'Week 12: VLA Applications', type: 'week', path: '/docs/weeks/week-12' },
          { id: 'week-13', title: 'Week 13: Course Review and Projects', type: 'week', path: '/docs/weeks/week-13' }
        ]
      }
    ];

    // Simulate API call delay
    setTimeout(() => {
      setCourseStructure(mockCourseStructure);
      setLoading(false);
    }, 300);
  }, []);

  const isActivePath = (path) => {
    return currentPath === path || currentPath.startsWith(path);
  };

  if (loading) {
    return (
      <div className={styles.navigationSidebar}>
        <div className={styles.loading}>
          <p>Loading course navigation...</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.navigationSidebar}>
      <div className={styles.courseTitle}>
        <h3>Course Structure</h3>
        <p>13-Week Humanoid Robotics Program</p>
      </div>

      <nav className={styles.navMenu}>
        {courseStructure.map((module) => (
          <div key={module.id} className={styles.moduleSection}>
            <div className={styles.moduleHeader}>
              <h4 className={styles.moduleTitle}>
                <Link
                  to={useBaseUrl(`/docs/module-${module.id.split('-')[1]}`)}
                  className={isActivePath(`/docs/module-${module.id.split('-')[1]}`) ? styles.active : ''}
                >
                  {module.title}
                </Link>
              </h4>
            </div>

            <ul className={styles.weekList}>
              {module.items.map((item) => (
                <li key={item.id} className={styles.weekItem}>
                  <Link
                    to={useBaseUrl(item.path)}
                    className={`${styles.weekLink} ${isActivePath(item.path) ? styles.active : ''}`}
                  >
                    {item.title}
                  </Link>
                </li>
              ))}
            </ul>
          </div>
        ))}
      </nav>

      <div className={styles.progressSection}>
        <h4>Your Progress</h4>
        <div className={styles.progressBar}>
          <div className={styles.progressFill} style={{ width: '25%' }}></div>
        </div>
        <p>Module 1, Week 2 of 13 weeks completed</p>
      </div>

      <div className={styles.resources}>
        <h4>Quick Links</h4>
        <ul className={styles.resourcesList}>
          <li><Link to={useBaseUrl('/docs/intro')}>Introduction</Link></li>
          <li><Link to={useBaseUrl('/docs/resources')}>Resources</Link></li>
          <li><Link to={useBaseUrl('/exercises')}>All Exercises</Link></li>
          <li><Link to={useBaseUrl('/simulations')}>Simulations</Link></li>
          <li><Link to={useBaseUrl('/chat')}>AI Assistant</Link></li>
        </ul>
      </div>
    </div>
  );
}

export default NavigationSidebar;