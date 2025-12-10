import logging
from typing import List, Optional, Dict, Any
from sqlalchemy.orm import Session
from sqlalchemy import func
from src.models import StudentProgress, User, CourseModule, WeeklyContent
from uuid import UUID

logger = logging.getLogger(__name__)

class ProgressService:
    def __init__(self, db: Session):
        self.db = db

    def get_user_progress(self, user_id: str) -> Dict[str, Any]:
        """Get a user's overall progress through the course"""
        try:
            # Get all progress records for the user
            progress_records = self.db.query(StudentProgress).filter(
                StudentProgress.user_id == UUID(user_id)
            ).all()

            # Calculate overall progress
            completed_modules = set()
            completed_weeks = set()
            completed_exercises = 0
            total_exercises = 0  # This would need to be calculated from the exercises table

            for record in progress_records:
                if record.status == "completed":
                    if record.module_id:
                        completed_modules.add(str(record.module_id))
                    if record.week_id:
                        completed_weeks.add(str(record.week_id))
                    if record.exercise_id:
                        completed_exercises += 1

            # Get user info to calculate progress percentage
            user = self.db.query(User).filter(User.id == UUID(user_id)).first()

            # Get total modules and weeks
            total_modules = self.db.query(CourseModule).count()
            total_weeks = self.db.query(WeeklyContent).count()

            progress_data = {
                "user_id": user_id,
                "completed_modules": len(completed_modules),
                "total_modules": total_modules,
                "completed_weeks": len(completed_weeks),
                "total_weeks": total_weeks,
                "completed_exercises": completed_exercises,
                "completed_percentage": 0,  # Calculate based on actual progress
                "current_week": user.current_week if user else 1,
                "detailed_progress": []
            }

            # Calculate percentage based on completed modules and weeks
            if total_modules > 0:
                module_progress = (len(completed_modules) / total_modules) * 60  # 60% for modules
                week_progress = (len(completed_weeks) / total_weeks) * 40      # 40% for weeks
                progress_data["completed_percentage"] = round(module_progress + week_progress, 2)

            # Update user's progress percentage in the database
            if user:
                user.progress_percentage = progress_data["completed_percentage"]
                self.db.commit()

            return progress_data
        except Exception as e:
            logger.error(f"Error getting user progress: {e}")
            return {}

    def update_user_progress(self, user_id: str, module_id: Optional[str] = None,
                           week_id: Optional[str] = None, exercise_id: Optional[str] = None,
                           status: str = "in_progress", score: Optional[float] = None) -> bool:
        """Update a user's progress for a specific module, week, or exercise"""
        try:
            # Check if progress record already exists
            query = self.db.query(StudentProgress).filter(
                StudentProgress.user_id == UUID(user_id)
            )

            if module_id:
                query = query.filter(StudentProgress.module_id == UUID(module_id))
            if week_id:
                query = query.filter(StudentProgress.week_id == UUID(week_id))
            if exercise_id:
                query = query.filter(StudentProgress.exercise_id == UUID(exercise_id))

            progress = query.first()

            if not progress:
                # Create new progress record
                progress = StudentProgress(
                    user_id=UUID(user_id),
                    module_id=UUID(module_id) if module_id else None,
                    week_id=UUID(week_id) if week_id else None,
                    exercise_id=UUID(exercise_id) if exercise_id else None,
                    status=status,
                    score=score
                )
                self.db.add(progress)
            else:
                # Update existing progress record
                progress.status = status
                if score is not None:
                    progress.score = score

            self.db.commit()
            return True
        except Exception as e:
            logger.error(f"Error updating user progress: {e}")
            self.db.rollback()
            return False

    def get_module_progress(self, user_id: str, module_id: str) -> Dict[str, Any]:
        """Get a user's progress for a specific module"""
        try:
            progress_records = self.db.query(StudentProgress).filter(
                StudentProgress.user_id == UUID(user_id),
                StudentProgress.module_id == UUID(module_id)
            ).all()

            completed_exercises = 0
            total_exercises = 0  # This would be calculated from exercises linked to this module

            for record in progress_records:
                if record.exercise_id and record.status == "completed":
                    completed_exercises += 1

            return {
                "user_id": user_id,
                "module_id": module_id,
                "completed_exercises": completed_exercises,
                "total_exercises": total_exercises,
                "status": "in_progress"  # Determine based on completion
            }
        except Exception as e:
            logger.error(f"Error getting module progress: {e}")
            return {}

    def get_week_progress(self, user_id: str, week_id: str) -> Dict[str, Any]:
        """Get a user's progress for a specific week"""
        try:
            progress_records = self.db.query(StudentProgress).filter(
                StudentProgress.user_id == UUID(user_id),
                StudentProgress.week_id == UUID(week_id)
            ).all()

            completed_exercises = 0
            total_exercises = 0  # This would be calculated from exercises linked to this week

            for record in progress_records:
                if record.exercise_id and record.status == "completed":
                    completed_exercises += 1

            return {
                "user_id": user_id,
                "week_id": week_id,
                "completed_exercises": completed_exercises,
                "total_exercises": total_exercises,
                "status": "in_progress"  # Determine based on completion
            }
        except Exception as e:
            logger.error(f"Error getting week progress: {e}")
            return {}