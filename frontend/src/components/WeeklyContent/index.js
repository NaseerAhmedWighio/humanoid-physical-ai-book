import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

function WeeklyContent({ weekId }) {
  const [week, setWeek] = useState(null);
  const [exercises, setExercises] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // In a real implementation, this would fetch from the backend API
    // For now, we'll use mock data
    const mockWeek = {
      id: weekId || '1',
      week_number: 1,
      title: 'Week 1: ROS 2 Architecture Overview',
      module_id: '1',
      subtopics: [
        'ROS 2 vs ROS 1',
        'DDS Communication',
        'Nodes and Packages'
      ],
      content_path: '/docs/weeks/week-01',
      exercises_count: 3,
      quizzes_count: 1,
      case_studies_count: 1,
      created_at: '2025-01-15T00:00:00Z'
    };

    const mockExercises = [
      {
        id: '1',
        title: 'ROS 2 Node Creation',
        type: 'coding',
        difficulty: 'beginner',
        content: 'Create a simple ROS 2 node that publishes a message to a topic',
        module_id: '1',
        week_id: '1',
        created_at: '2025-01-15T00:00:00Z'
      },
      {
        id: '2',
        title: 'Topic Publisher',
        type: 'coding',
        difficulty: 'intermediate',
        content: 'Implement a publisher that sends sensor data to a topic',
        module_id: '1',
        week_id: '1',
        created_at: '2025-01-15T00:00:00Z'
      }
    ];

    // Simulate API call delay
    setTimeout(() => {
      setWeek(mockWeek);
      setExercises(mockExercises);
      setLoading(false);
    }, 500);
  }, [weekId]);

  if (loading) {
    return (
      <div className={styles.weeklyContent}>
        <h2>Loading Weekly Content...</h2>
        <p>Loading week details from the backend API...</p>
      </div>
    );
  }

  if (!week) {
    return (
      <div className={styles.weeklyContent}>
        <h2>Week Not Found</h2>
        <p>The requested week could not be found.</p>
        <Link to={useBaseUrl('/docs/intro')}>Return to Introduction</Link>
      </div>
    );
  }

  return (
    <div className={styles.weeklyContent}>
      <header className={styles.weekHeader}>
        <div className={styles.weekTitleSection}>
          <span className={styles.weekNumber}>Week {week.week_number}</span>
          <h1>{week.title}</h1>
        </div>

        <div className={styles.weekMeta}>
          <div className={styles.metaItem}>
            <strong>Module:</strong> {week.module_id}
          </div>
          <div className={styles.metaItem}>
            <strong>Exercises:</strong> {week.exercises_count}
          </div>
          <div className={styles.metaItem}>
            <strong>Quizzes:</strong> {week.quizzes_count}
          </div>
          {week.case_studies_count > 0 && (
            <div className={styles.metaItem}>
              <strong>Case Studies:</strong> {week.case_studies_count}
            </div>
          )}
        </div>
      </header>

      <section className={styles.subtopicsSection}>
        <h2>Subtopics</h2>
        <ul className={styles.subtopicsList}>
          {week.subtopics.map((subtopic, index) => (
            <li key={index} className={styles.subtopicItem}>
              <span className={styles.subtopicBullet}>•</span>
              {subtopic}
            </li>
          ))}
        </ul>
      </section>

      <section className={styles.exercisesSection}>
        <h2>Exercises</h2>
        {exercises.length > 0 ? (
          <div className={styles.exercisesGrid}>
            {exercises.map((exercise) => (
              <div key={exercise.id} className={styles.exerciseCard}>
                <div className={styles.exerciseHeader}>
                  <h3>{exercise.title}</h3>
                  <span className={`${styles.difficultyBadge} ${styles[exercise.difficulty]}`}>
                    {exercise.difficulty}
                  </span>
                </div>

                <p className={styles.exerciseContent}>{exercise.content}</p>

                <div className={styles.exerciseMeta}>
                  <span className={styles.exerciseType}>{exercise.type}</span>
                </div>

                <div className={styles.exerciseActions}>
                  <Link
                    className="button button--primary button--sm"
                    to={useBaseUrl(`/exercises/${exercise.id}`)}
                  >
                    Start Exercise
                  </Link>
                </div>
              </div>
            ))}
          </div>
        ) : (
          <p>No exercises available for this week yet.</p>
        )}
      </section>

      <section className={styles.resourcesSection}>
        <h2>Learning Resources</h2>
        <ul>
          <li>
            <Link to={useBaseUrl(week.content_path)}>Week Content</Link>
          </li>
          <li>
            <Link to={useBaseUrl(`/docs/module-${week.module_id}`)}>Module Overview</Link>
          </li>
          <li>
            <Link to={useBaseUrl('/docs/resources')}>Additional Resources</Link>
          </li>
        </ul>
      </section>

      <section className={styles.weekNavigation}>
        <div className={styles.navButton}>
          <Link
            className="button button--outline button--primary"
            to={useBaseUrl(`/docs/weeks/${parseInt(week.week_number) - 1}`)}
            disabled={week.week_number <= 1}
          >
            Previous Week
          </Link>
        </div>
        <div className={styles.navButton}>
          <Link
            className="button button--primary"
            to={useBaseUrl(`/docs/weeks/${parseInt(week.week_number) + 1}`)}
          >
            Next Week
          </Link>
        </div>
      </section>
    </div>
  );
}

export default WeeklyContent;