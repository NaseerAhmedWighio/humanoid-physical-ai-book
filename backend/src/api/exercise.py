from fastapi import APIRouter, HTTPException, Depends
from typing import List, Optional
from uuid import UUID
from sqlalchemy.orm import Session
from src.database import get_db
from src.services.exercise_service import ExerciseService

router = APIRouter()

@router.get("/exercises")
async def get_exercises(module_id: Optional[str] = None, week_id: Optional[str] = None, db: Session = Depends(get_db)):
    """
    Get exercises, optionally filtered by module or week
    """
    exercise_service = ExerciseService(db)

    if module_id:
        exercises = exercise_service.get_exercises_by_module(module_id)
    elif week_id:
        exercises = exercise_service.get_exercises_by_week(week_id)
    else:
        # Return empty list if no filter is provided
        exercises = []

    return {"exercises": exercises}

@router.get("/exercises/{exercise_id}")
async def get_exercise(exercise_id: str, db: Session = Depends(get_db)):
    """
    Get a specific exercise by ID
    """
    exercise_service = ExerciseService(db)
    exercise = exercise_service.get_exercise_by_id(exercise_id)
    if not exercise:
        raise HTTPException(status_code=404, detail="Exercise not found")
    return exercise

@router.post("/exercises/{exercise_id}/submit")
async def submit_exercise(exercise_id: str, answer: str, user_id: str, db: Session = Depends(get_db)):
    """
    Submit an answer for an exercise
    """
    exercise_service = ExerciseService(db)
    result = exercise_service.submit_exercise(user_id, exercise_id, answer)

    if not result["success"]:
        raise HTTPException(status_code=400, detail=result.get("error", "Submission failed"))

    return result

@router.get("/exercises/{exercise_id}/progress")
async def get_exercise_progress(exercise_id: str, user_id: str, db: Session = Depends(get_db)):
    """
    Get a user's progress on a specific exercise
    """
    exercise_service = ExerciseService(db)
    progress = exercise_service.get_user_exercise_progress(user_id, exercise_id)
    if not progress:
        return {"progress": None}
    return {"progress": progress}