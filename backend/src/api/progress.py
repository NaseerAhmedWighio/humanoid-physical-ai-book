from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any
from uuid import UUID
from sqlalchemy.orm import Session
from src.database import get_db
from src.services.progress_service import ProgressService

router = APIRouter()

@router.get("/users/{user_id}/progress")
async def get_user_progress(user_id: str, db: Session = Depends(get_db)):
    """Get user's progress through the course"""
    progress_service = ProgressService(db)
    progress = progress_service.get_user_progress(user_id)
    if not progress:
        raise HTTPException(status_code=404, detail="User progress not found")
    return progress

@router.put("/users/{user_id}/progress")
async def update_user_progress(
    user_id: str,
    module_id: str = None,
    week_id: str = None,
    exercise_id: str = None,
    status: str = "in_progress",
    score: float = None,
    db: Session = Depends(get_db)
):
    """Update user's progress"""
    progress_service = ProgressService(db)
    success = progress_service.update_user_progress(
        user_id=user_id,
        module_id=module_id,
        week_id=week_id,
        exercise_id=exercise_id,
        status=status,
        score=score
    )
    if not success:
        raise HTTPException(status_code=400, detail="Failed to update progress")
    return {"user_id": user_id, "updated": True}