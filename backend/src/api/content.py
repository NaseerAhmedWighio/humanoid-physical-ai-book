from fastapi import APIRouter, HTTPException, Depends
from typing import List, Optional
from uuid import UUID
from sqlalchemy.orm import Session
from src.database import get_db
from src.services.content_service import ContentService

router = APIRouter()

@router.get("/modules")
async def get_modules(db: Session = Depends(get_db)):
    """
    Get all course modules
    """
    content_service = ContentService(db)
    modules = content_service.get_all_modules()
    return {"modules": modules}

@router.get("/modules/{module_id}")
async def get_module(module_id: str, db: Session = Depends(get_db)):
    """
    Get a specific course module
    """
    content_service = ContentService(db)
    module = content_service.get_module_by_id(module_id)
    if not module:
        raise HTTPException(status_code=404, detail="Module not found")
    return module

@router.get("/weeks")
async def get_weeks(db: Session = Depends(get_db)):
    """
    Get all weekly content
    """
    content_service = ContentService(db)
    weeks = content_service.get_all_weeks()
    return {"weeks": weeks}

@router.get("/weeks/{week_id}")
async def get_week(week_id: str, db: Session = Depends(get_db)):
    """
    Get a specific week's content
    """
    content_service = ContentService(db)
    week = content_service.get_week_by_id(week_id)
    if not week:
        raise HTTPException(status_code=404, detail="Week not found")
    return week

@router.post("/search")
async def search_content(query: str, limit: int = 10, db: Session = Depends(get_db)):
    """
    Search content using RAG functionality
    """
    content_service = ContentService(db)
    results = content_service.search_content(query, limit)
    return {"query": query, "results": results, "limit": limit}