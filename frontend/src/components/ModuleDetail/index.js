import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

function ModuleDetail({ moduleId }) {
  const [module, setModule] = useState(null);
  const [weeklyContent, setWeeklyContent] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // In a real implementation, this would fetch from the backend API
    // For now, we'll use mock data
    const mockModule = {
      id: moduleId || '1',
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
    };

    // Mock weekly content for this module
    const mockWeeklyContent = [
      {
        id: '1',
        week_number: 1,
        title: 'Week 1: ROS 2 Architecture Overview',
        subtopics: ['ROS 2 vs ROS 1', 'DDS Communication', 'Nodes and Packages'],
        content_path: '/docs/weeks/week-01',
        exercises_count: 3,
        quizzes_count: 1,
        case_studies_count: 1,
        created_at: '2025-01-15T00:00:00Z'
      },
      {
        id: '2',
        week_number: 2,
        title: 'Week 2: Nodes, Topics, Services',
        subtopics: ['Creating Nodes', 'Publishing and Subscribing', 'Services'],
        content_path: '/docs/weeks/week-02',
        exercises_count: 4,
        quizzes_count: 1,
        case_studies_count: 0,
        created_at: '2025-01-15T00:00:00Z'
      }
    ];

    // Simulate API call delay
    setTimeout(() => {
      setModule(mockModule);
      setWeeklyContent(mockWeeklyContent);
      setLoading(false);
    }, 500);
  }, [moduleId]);

  if (loading) {
    return (
      <div className={styles.moduleDetail}>
        <h2>Loading Module Details...</h2>
        <p>Loading module content from the backend API...</p>
      </div>
    );
  }

  if (!module) {
    return (
      <div className={styles.moduleDetail}>
        <h2>Module Not Found</h2>
        <p>The requested module could not be found.</p>
        <Link to={useBaseUrl('/docs/intro')}>Return to Introduction</Link>
      </div>
    );
  }

  return (
    <div className={styles.moduleDetail}>
      <header className={styles.moduleHeader}>
        <div className={styles.moduleTitleSection}>
          <span className={styles.moduleNumber}>Module {module.module_number}</span>
          <h1>{module.title}</h1>
        </div>

        <div className={styles.moduleMeta}>
          <div className={styles.metaItem}>
            <strong>Word Count:</strong> {module.word_count.toLocaleString()}
          </div>
          <div className={styles.metaItem}>
            <strong>Duration:</strong> ~{module.estimated_duration_hours} hours
          </div>
        </div>
      </header>

      <section className={styles.moduleDescription}>
        <h2>Description</h2>
        <p>{module.description}</p>
      </section>

      <section className={styles.learningOutcomes}>
        <h2>Learning Outcomes</h2>
        <ul>
          {module.learning_outcomes.map((outcome, index) => (
            <li key={index}>{outcome}</li>
          ))}
        </ul>
      </section>

      <section className={styles.weeklyContent}>
        <h2>Weekly Content</h2>
        <div className={styles.weeksGrid}>
          {weeklyContent.map((week) => (
            <div key={week.id} className={styles.weekCard}>
              <div className={styles.weekHeader}>
                <h3>
                  <Link to={useBaseUrl(week.content_path)}>
                    Week {week.week_number}: {week.title}
                  </Link>
                </h3>
              </div>

              <div className={styles.subtopics}>
                <h4>Subtopics:</h4>
                <ul>
                  {week.subtopics.map((subtopic, index) => (
                    <li key={index}>{subtopic}</li>
                  ))}
                </ul>
              </div>

              <div className={styles.weekMetrics}>
                <span className={styles.exercises}>{week.exercises_count} exercises</span>
                <span className={styles.quizzes}>{week.quizzes_count} quiz</span>
                {week.case_studies_count > 0 && (
                  <span className={styles.caseStudies}>{week.case_studies_count} case study</span>
                )}
              </div>

              <div className={styles.weekActions}>
                <Link
                  className="button button--primary button--sm"
                  to={useBaseUrl(week.content_path)}
                >
                  View Content
                </Link>
              </div>
            </div>
          ))}
        </div>
      </section>

      <section className={styles.moduleActions}>
        <Link
          className="button button--primary button--lg"
          to={useBaseUrl(module.content_path || `/docs/module-${module.module_number}`)}
        >
          Start Learning
        </Link>
      </section>
    </div>
  );
}

export default ModuleDetail;