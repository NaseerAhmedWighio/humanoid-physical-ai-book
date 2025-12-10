import logging
from typing import List, Optional, Dict, Any
from sqlalchemy.orm import Session
from sqlalchemy import func
from src.models import Exercise, StudentProgress
from uuid import UUID

logger = logging.getLogger(__name__)

class ExerciseService:
    def __init__(self, db: Session):
        self.db = db

    def get_exercises_by_module(self, module_id: str) -> List[Dict[str, Any]]:
        """Get all exercises for a specific module"""
        try:
            exercises = self.db.query(Exercise).filter(Exercise.module_id == UUID(module_id)).all()
            result = []
            for exercise in exercises:
                result.append({
                    "id": str(exercise.id),
                    "title": exercise.title,
                    "type": exercise.type,
                    "difficulty": exercise.difficulty,
                    "content": exercise.content,
                    "module_id": str(exercise.module_id),
                    "week_id": str(exercise.week_id) if exercise.week_id else None,
                    "created_at": exercise.created_at
                })
            return result
        except Exception as e:
            logger.error(f"Error getting exercises by module: {e}")
            return []

    def get_exercises_by_week(self, week_id: str) -> List[Dict[str, Any]]:
        """Get all exercises for a specific week"""
        try:
            exercises = self.db.query(Exercise).filter(Exercise.week_id == UUID(week_id)).all()
            result = []
            for exercise in exercises:
                result.append({
                    "id": str(exercise.id),
                    "title": exercise.title,
                    "type": exercise.type,
                    "difficulty": exercise.difficulty,
                    "content": exercise.content,
                    "module_id": str(exercise.module_id),
                    "week_id": str(exercise.week_id) if exercise.week_id else None,
                    "created_at": exercise.created_at
                })
            return result
        except Exception as e:
            logger.error(f"Error getting exercises by week: {e}")
            return []

    def get_exercise_by_id(self, exercise_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific exercise by ID"""
        try:
            exercise = self.db.query(Exercise).filter(Exercise.id == UUID(exercise_id)).first()
            if exercise:
                return {
                    "id": str(exercise.id),
                    "title": exercise.title,
                    "type": exercise.type,
                    "difficulty": exercise.difficulty,
                    "content": exercise.content,
                    "solution": exercise.solution,  # This would not be returned in a real implementation for security
                    "module_id": str(exercise.module_id),
                    "week_id": str(exercise.week_id) if exercise.week_id else None,
                    "created_at": exercise.created_at,
                    "updated_at": exercise.updated_at
                }
            return None
        except Exception as e:
            logger.error(f"Error getting exercise by ID: {e}")
            return None

    def submit_exercise(self, user_id: str, exercise_id: str, answer: str) -> Dict[str, Any]:
        """Submit an answer for an exercise and grade it"""
        try:
            # Get the exercise
            exercise = self.db.query(Exercise).filter(Exercise.id == UUID(exercise_id)).first()
            if not exercise:
                return {"success": False, "error": "Exercise not found"}

            # Grade the exercise (simplified grading logic)
            # In a real implementation, this would involve more complex logic based on exercise type
            is_correct = self._grade_answer(exercise.solution, answer)

            # Create or update student progress
            progress = self.db.query(StudentProgress).filter(
                StudentProgress.user_id == UUID(user_id),
                StudentProgress.exercise_id == UUID(exercise_id)
            ).first()

            if not progress:
                # Create new progress record
                progress = StudentProgress(
                    user_id=UUID(user_id),
                    exercise_id=UUID(exercise_id),
                    module_id=exercise.module_id,
                    week_id=exercise.week_id,
                    status="completed" if is_correct else "in_progress",
                    score=100.0 if is_correct else 0.0,
                    attempts_count=1
                )
                self.db.add(progress)
            else:
                # Update existing progress
                progress.status = "completed" if is_correct else "in_progress"
                progress.score = 100.0 if is_correct else 0.0
                progress.attempts_count += 1
                if is_correct and progress.status != "completed":
                    progress.completed_at = func.now()

            self.db.commit()

            return {
                "success": True,
                "is_correct": is_correct,
                "score": 100.0 if is_correct else 0.0,
                "attempts_count": progress.attempts_count
            }
        except Exception as e:
            logger.error(f"Error submitting exercise: {e}")
            self.db.rollback()
            return {"success": False, "error": str(e)}

    def _grade_answer(self, expected_solution: str, user_answer: str) -> bool:
        """Grade the user's answer against the expected solution"""
        # This is a simplified grading implementation
        # In a real system, this would vary based on exercise type:
        # - For coding exercises: run tests against the code
        # - For conceptual exercises: use NLP or keyword matching
        # - For multiple choice: direct comparison
        # For now, we'll do a simple string comparison (case-insensitive)

        return expected_solution.lower().strip() == user_answer.lower().strip()

    def get_user_exercise_progress(self, user_id: str, exercise_id: str) -> Optional[Dict[str, Any]]:
        """Get a user's progress on a specific exercise"""
        try:
            progress = self.db.query(StudentProgress).filter(
                StudentProgress.user_id == UUID(user_id),
                StudentProgress.exercise_id == UUID(exercise_id)
            ).first()

            if progress:
                return {
                    "id": str(progress.id),
                    "user_id": str(progress.user_id),
                    "exercise_id": str(progress.exercise_id),
                    "status": progress.status,
                    "score": progress.score,
                    "attempts_count": progress.attempts_count,
                    "last_accessed": progress.last_accessed,
                    "completed_at": progress.completed_at,
                    "created_at": progress.created_at
                }
            return None
        except Exception as e:
            logger.error(f"Error getting user exercise progress: {e}")
            return None