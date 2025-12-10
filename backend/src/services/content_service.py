import logging
from typing import List, Optional, Dict, Any
from sqlalchemy.orm import Session
from src.models import CourseModule, WeeklyContent, ContentChunk
from src.services.rag_service import rag_service
from uuid import UUID

logger = logging.getLogger(__name__)

class ContentService:
    def __init__(self, db: Session):
        self.db = db

    def get_all_modules(self) -> List[Dict[str, Any]]:
        """Get all course modules"""
        try:
            modules = self.db.query(CourseModule).all()
            result = []
            for module in modules:
                result.append({
                    "id": str(module.id),
                    "title": module.title,
                    "module_number": module.module_number,
                    "description": module.description,
                    "word_count": module.word_count,
                    "estimated_duration_hours": module.estimated_duration_hours,
                    "learning_outcomes": module.learning_outcomes,
                    "created_at": module.created_at
                })
            return result
        except Exception as e:
            logger.error(f"Error getting all modules: {e}")
            return []

    def get_module_by_id(self, module_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific course module by ID"""
        try:
            module = self.db.query(CourseModule).filter(CourseModule.id == UUID(module_id)).first()
            if module:
                return {
                    "id": str(module.id),
                    "title": module.title,
                    "module_number": module.module_number,
                    "description": module.description,
                    "word_count": module.word_count,
                    "estimated_duration_hours": module.estimated_duration_hours,
                    "learning_outcomes": module.learning_outcomes,
                    "prerequisites": module.prerequisites,
                    "created_at": module.created_at,
                    "updated_at": module.updated_at
                }
            return None
        except Exception as e:
            logger.error(f"Error getting module by ID: {e}")
            return None

    def get_all_weeks(self) -> List[Dict[str, Any]]:
        """Get all weekly content"""
        try:
            weeks = self.db.query(WeeklyContent).order_by(WeeklyContent.week_number).all()
            result = []
            for week in weeks:
                result.append({
                    "id": str(week.id),
                    "week_number": week.week_number,
                    "title": week.title,
                    "module_id": str(week.module_id),
                    "subtopics": week.subtopics,
                    "content_path": week.content_path,
                    "exercises_count": week.exercises_count,
                    "quizzes_count": week.quizzes_count,
                    "case_studies_count": week.case_studies_count,
                    "created_at": week.created_at
                })
            return result
        except Exception as e:
            logger.error(f"Error getting all weeks: {e}")
            return []

    def get_week_by_id(self, week_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific week's content by ID"""
        try:
            week = self.db.query(WeeklyContent).filter(WeeklyContent.id == UUID(week_id)).first()
            if week:
                return {
                    "id": str(week.id),
                    "week_number": week.week_number,
                    "title": week.title,
                    "module_id": str(week.module_id),
                    "subtopics": week.subtopics,
                    "content_path": week.content_path,
                    "exercises_count": week.exercises_count,
                    "quizzes_count": week.quizzes_count,
                    "case_studies_count": week.case_studies_count,
                    "created_at": week.created_at,
                    "updated_at": week.updated_at
                }
            return None
        except Exception as e:
            logger.error(f"Error getting week by ID: {e}")
            return None

    def search_content(self, query: str, limit: int = 10) -> List[Dict[str, Any]]:
        """Search content using RAG functionality"""
        # In a real implementation, we would use an embedding model to convert the query to a vector
        # For now, we'll use a mock implementation
        try:
            # This is a simplified implementation - in reality, you'd:
            # 1. Use OpenAI or another service to get embeddings for the query
            # 2. Call rag_service.search_content with the embedding
            # For now, we'll return empty results to be implemented later
            mock_results = []
            logger.info(f"Search query: {query}, limit: {limit}")
            return mock_results
        except Exception as e:
            logger.error(f"Error searching content: {e}")
            return []

    def get_content_chunks_by_module(self, module_id: str) -> List[Dict[str, Any]]:
        """Get all content chunks for a specific module"""
        try:
            chunks = self.db.query(ContentChunk).filter(ContentChunk.module_id == UUID(module_id)).all()
            result = []
            for chunk in chunks:
                result.append({
                    "id": str(chunk.id),
                    "chunk_index": chunk.chunk_index,
                    "content": chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content,  # Truncate for display
                    "semantic_tags": chunk.semantic_tags,
                    "created_at": chunk.created_at
                })
            return result
        except Exception as e:
            logger.error(f"Error getting content chunks by module: {e}")
            return []