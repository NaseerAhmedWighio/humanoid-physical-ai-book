from fastapi import APIRouter
from typing import Dict, Any

router = APIRouter()

@router.post("/sessions")
async def create_chat_session():
    """Create a new chat session"""
    # This will be implemented later
    return {"session_id": "sample-session-id", "title": "New Chat Session"}

@router.post("/sessions/{session_id}/messages")
async def send_chat_message(session_id: str, content: str):
    """Send a message in a chat session"""
    # This will be implemented later with OpenAI integration
    return {"session_id": session_id, "response": "Sample response"}