# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Overview
This guide will help you set up and run the AI-native textbook website for Physical AI & Humanoid Robotics. The system consists of a Docusaurus frontend for content delivery and a FastAPI backend for APIs, RAG functionality, and chat services.

## Prerequisites
- Python 3.8+
- Node.js 16+
- PostgreSQL 12+
- Docker (for optional containerized setup)
- OpenAI API key
- Qdrant vector database (local or cloud)

## Backend Setup

### 1. Environment Setup
```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install backend dependencies
cd backend
pip install -r requirements.txt
```

### 2. Environment Variables
Create a `.env` file in the backend directory:
```env
DATABASE_URL=postgresql://username:password@localhost/dbname
QDRANT_URL=http://localhost:6333
OPENAI_API_KEY=your_openai_api_key
SECRET_KEY=your_secret_key
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
```

### 3. Database Setup
```bash
# Run database migrations
alembic upgrade head

# Initialize content (run content generation script)
python scripts/generate_content.py
```

### 4. Vector Database Setup
```bash
# Populate vector database with content chunks
python scripts/populate_vector_db.py
```

### 5. Run Backend Server
```bash
# Start the backend server
uvicorn src.main:app --reload --port 8000
```

## Frontend Setup

### 1. Install Dependencies
```bash
cd frontend
npm install
```

### 2. Environment Variables
Create a `.env` file in the frontend directory:
```env
API_BASE_URL=http://localhost:8000
CHAT_ENABLED=true
SEARCH_ENABLED=true
```

### 3. Run Development Server
```bash
npm start
```

## Content Generation

### Generate Course Modules
The system includes a content generation script that creates the 4 main modules and 13 weeks of content:

```bash
# Generate all content (3000-5000+ words per module)
python scripts/generate_content.py --all

# Generate specific module
python scripts/generate_content.py --module 1

# Validate generated content
python scripts/validate_content.py
```

### Content Structure
Generated content will be placed in `frontend/docs/`:
```
docs/
├── intro/
│   ├── index.md
│   └── why-physical-ai-matters.md
├── module1-ros2/
│   ├── index.md
│   ├── nodes-topics-services-actions.md
│   ├── rclpy-examples.md
│   └── urdf-bipedal-models.md
├── module2-gazebo-unity/
├── module3-nvidia-isaac/
├── module4-vla/
├── weeks/
│   ├── week01-02-introduction.md
│   ├── week03-05-ros2-fundamentals.md
│   └── ...
├── assessments/
└── appendices/
```

## API Usage

### Content Endpoints
```bash
# Get all modules
curl http://localhost:8000/v1/content/modules

# Get specific module
curl http://localhost:8000/v1/content/modules/{module_id}

# Search content using RAG
curl -X POST http://localhost:8000/v1/content/search \
  -H "Content-Type: application/json" \
  -d '{"query": "How do I create a ROS 2 node?"}'
```

### Exercise Endpoints
```bash
# Get exercises for a module
curl "http://localhost:8000/v1/exercises?module_id={module_id}"

# Submit exercise answer
curl -X POST http://localhost:8000/v1/exercises/{exercise_id}/submit \
  -H "Content-Type: application/json" \
  -d '{"answer": "My answer here"}'
```

### Chat Endpoints
```bash
# Create a chat session
curl -X POST http://localhost:8000/v1/chat/sessions

# Send message to chat
curl -X POST http://localhost:8000/v1/chat/sessions/{session_id}/messages \
  -H "Content-Type: application/json" \
  -d '{"content": "Explain ROS 2 nodes"}'
```

## Docusaurus Custom Components

The frontend includes custom components for interactive learning:

### Code Blocks with Execution
```md
import CodeBlock from '@site/src/components/CodeBlock';

<CodeBlock language="python">
# ROS 2 Publisher Example
import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('publisher_node')
    publisher = node.create_publisher(String, 'topic', 10)
    # ...
</CodeBlock>
```

### Interactive Quizzes
```md
import InteractiveQuiz from '@site/src/components/InteractiveQuiz';

<InteractiveQuiz
  question="What is the main purpose of ROS 2?"
  options={["Robot Operating System", "Robot Operating Service", "Robot Operation System", "None of the above"]}
  correct={0}
/>
```

### Mermaid Diagrams
```md
import MermaidDiagram from '@site/src/components/MermaidDiagram';

<MermaidDiagram>
graph LR
    A[ROS 2 Node] --> B[Topic]
    B --> C[ROS 2 Node]
</MermaidDiagram>
```

## Running Tests

### Backend Tests
```bash
# Run all backend tests
python -m pytest tests/

# Run specific test suite
python -m pytest tests/unit/
python -m pytest tests/integration/
```

### Frontend Tests
```bash
# Run frontend tests
npm test
npm run build  # Build for production
```

## Deployment

### Production Build
```bash
# Build backend for production
cd backend
pip install -r requirements-prod.txt

# Build frontend for production
cd frontend
npm run build
```

### Docker Deployment
```bash
# Build and run with Docker Compose
docker-compose up --build
```

## Troubleshooting

### Common Issues
1. **Content not loading**: Check that the content generation script has been run
2. **RAG search not working**: Verify Qdrant is running and populated with content
3. **Chat not responding**: Check OpenAI API key and connectivity
4. **Exercises not grading**: Verify solution formats in the database

### Logs
- Backend logs: Check console output from `uvicorn` command
- Frontend logs: Check browser console and Docusaurus build logs
- Database: Check PostgreSQL logs for connection issues
- Vector DB: Check Qdrant logs for indexing issues