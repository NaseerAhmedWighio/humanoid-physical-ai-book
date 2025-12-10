import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

function ModuleList() {
  const [modules, setModules] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // In a real implementation, this would fetch from the backend API
    // For now, we'll use mock data
    const mockModules = [
      {
        id: '1',
        title: 'Introduction to ROS 2',
        module_number: 1,
        description: 'Fundamentals of Robot Operating System 2 for humanoid robotics applications',
        word_count: 4000,
        estimated_duration_hours: 10.0,
        learning_outcomes: [
          'Understand ROS 2 architecture and concepts',
          'Create ROS 2 nodes, topics, services, and actions',
          'Work with URDF for humanoid robot models'
        ],
        created_at: '2025-01-15T00:00:00Z'
      },
      {
        id: '2',
        title: 'Simulation with Gazebo and Unity',
        module_number: 2,
        description: 'Simulation environments for testing humanoid robots',
        word_count: 4200,
        estimated_duration_hours: 12.0,
        learning_outcomes: [
          'Set up Gazebo simulation environments',
          'Create Unity scenes for robot simulation',
          'Implement physics-based robot control'
        ],
        created_at: '2025-01-16T00:00:00Z'
      },
      {
        id: '3',
        title: 'NVIDIA Isaac for Humanoid Control',
        module_number: 3,
        description: 'Using NVIDIA Isaac platform for advanced humanoid robotics',
        word_count: 4500,
        estimated_duration_hours: 14.0,
        learning_outcomes: [
          'Implement perception systems using Isaac',
          'Develop control algorithms for humanoid robots',
          'Optimize performance with NVIDIA hardware'
        ],
        created_at: '2025-01-17T00:00:00Z'
      },
      {
        id: '4',
        title: 'Vision Language Action (VLA) Models',
        module_number: 4,
        description: 'Understanding and implementing VLA models for robot interaction',
        word_count: 4800,
        estimated_duration_hours: 16.0,
        learning_outcomes: [
          'Understand VLA model architecture',
          'Implement vision-language integration',
          'Apply VLA models to humanoid tasks'
        ],
        created_at: '2025-01-18T00:00:00Z'
      }
    ];

    // Simulate API call delay
    setTimeout(() => {
      setModules(mockModules);
      setLoading(false);
    }, 500);
  }, []);

  if (loading) {
    return (
      <div className={styles.moduleList}>
        <h2>Loading Modules...</h2>
        <p>Loading course modules from the backend API...</p>
      </div>
    );
  }

  return (
    <div className={styles.moduleList}>
      <h2>Course Modules</h2>
      <p>Browse through our comprehensive modules on Physical AI & Humanoid Robotics</p>

      <div className="flex justify-between items-center ">
        {modules.map((module) => (
          <div key={module.id} >
            <div className={styles.moduleHeader}>
              <h3>
                <Link to={useBaseUrl(`/docs/module-${module.module_number}`)}>
                  Module {module.module_number}: {module.title}
                </Link>
              </h3>
              <span className={styles.moduleNumber}>#{module.module_number}</span>
            </div>

            <p className={styles.moduleDescription}>{module.description}</p>

            <div className={styles.moduleMeta}>
              <span className={styles.wordCount}>{module.word_count.toLocaleString()} words</span>
              <span className={styles.duration}>{module.estimated_duration_hours} hours</span>
            </div>

            <div className={styles.learningOutcomes}>
              <h4>Learning Outcomes:</h4>
              <ul>
                {module.learning_outcomes.map((outcome, index) => (
                  <li key={index}>{outcome}</li>
                ))}
              </ul>
            </div>

            <div className={styles.moduleActions}>
              <Link
                className="button button--primary button--lg"
                to={useBaseUrl(`/docs/module-${module.module_number}`)}
              >
                Start Module
              </Link>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

export default ModuleList;