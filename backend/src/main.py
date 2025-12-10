from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="Backend API for the AI-native textbook website",
    version="0.1.0"
)

# Add CORS middleware for frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"message": "Welcome to the Physical AI & Humanoid Robotics Textbook API"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

# Include API routes
from .api import content, chat, progress, exercise, simulation
app.include_router(content.router, prefix="/v1/content", tags=["content"])
app.include_router(chat.router, prefix="/v1/chat", tags=["chat"])
app.include_router(progress.router, prefix="/v1/progress", tags=["progress"])
app.include_router(exercise.router, prefix="/v1/exercises", tags=["exercises"])
app.include_router(simulation.router, prefix="/v1/simulation", tags=["simulation"])