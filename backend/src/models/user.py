from sqlalchemy import Column, Integer, String, DateTime, Float
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from src.database import Base

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String, unique=True, nullable=False)
    name = Column(String, nullable=False)
    student_id = Column(String, nullable=True)  # Optional, for institutional tracking
    enrollment_date = Column(DateTime(timezone=True), server_default=func.now())
    current_week = Column(Integer, default=1)  # Default: 1
    progress_percentage = Column(Float, default=0.0)  # Default: 0.0
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())