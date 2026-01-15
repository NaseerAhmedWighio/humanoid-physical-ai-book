# Humanoid AI Book - Setup Instructions

This document provides comprehensive instructions to set up the Humanoid AI Book application with full RAG (Retrieval Augmented Generation) functionality.

## Prerequisites

- Python 3.8+
- Node.js 18+
- Docker (for Qdrant vector database)
- An API key from [OpenRouter](https://openrouter.ai/) (free tier available)

## Installation Steps

### 1. Clone and Set Up Backend

```bash
# Navigate to the backend directory
cd backend

# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Set Up Environment Variables

Create a `.env` file in the `backend` directory with the following content:

```env
# OpenRouter API Key for LLM access (get from https://openrouter.ai/)
OPENROUTER_API_KEY=your_openrouter_api_key_here

# Qdrant Vector Database Configuration
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=

# LLM Provider Configuration
LLM_PROVIDER=gemini
OPENROUTER_MODEL=mistralai/devstral-2512:free
TRANSLATION_MODEL=google/gemma-3-4b-it:free

# Database Configuration
DATABASE_URL=sqlite:///./humanoid_ai_book.db

# Authentication Configuration
JWT_SECRET=your_jwt_secret_here_change_this_in_production
BETTER_AUTH_SECRET=your_better_auth_secret_here

# CORS Configuration
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000,http://127.0.0.1:3000,http://127.0.0.1:8000
```

### 3. Set Up Qdrant Vector Database

Qdrant is required for RAG functionality. Choose one of the following methods:

#### Option A: Using Docker (Recommended)

```bash
# Pull and run Qdrant container
docker run -d --name qdrant-humanoid-ai -p 6333:6333 -p 6334:6334 qdrant/qdrant

# Verify Qdrant is running
curl http://localhost:6333/health
```

#### Option B: Using Docker Compose

Create a `docker-compose.yml` file:

```yaml
version: '3.8'
services:
  qdrant:
    image: qdrant/qdrant
    container_name: qdrant-humanoid-ai
    ports:
      - "6333:6333"
      - "6334:6334"
    volumes:
      - qdrant_data:/qdrant_data

volumes:
  qdrant_data:
```

Then run:
```bash
docker-compose up -d
```

#### Option C: Cloud Qdrant

Sign up at [Qdrant Cloud](https://cloud.qdrant.io/) and update your `.env` file:

```env
QDRANT_URL=https://your-cluster-url.qdrant.tech
QDRANT_API_KEY=your_cloud_api_key
```

### 4. Initialize Database and Content

```bash
# Navigate to backend
cd backend

# Initialize database tables
python scripts/init_db.py

# Create Qdrant collection and ingest sample content
python scripts/ingest_sample_content.py
```

### 5. Set Up Frontend

```bash
# Navigate to frontend
cd frontend

# Install dependencies
npm install
```

### 6. Start the Applications

#### Start Backend Server

```bash
cd backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

#### Start Frontend Server

```bash
cd frontend
npm run start
```

## Troubleshooting

### Common Issues

1. **Qdrant Connection Error**: Make sure the Qdrant server is running and accessible at `http://localhost:6333`

2. **API Key Issues**: Verify your OpenRouter API key is correct and has sufficient quota

3. **Database Connection Issues**: Ensure the database file path is correct and permissions are set properly

### Testing RAG Functionality

Use the CLI-based RAG tester:

```bash
cd backend
python scripts/rag_tester.py
```

### Docker Management

```bash
# Stop Qdrant container
docker stop qdrant-humanoid-ai

# Start Qdrant container
docker start qdrant-humanoid-ai

# Remove Qdrant container
docker rm qdrant-humanoid-ai

# View Qdrant logs
docker logs qdrant-humanoid-ai
```

## Additional Scripts

The following utility scripts are available in the `backend/scripts/` directory:

- `setup_qdrant.py` - Automated Qdrant setup
- `test_qdrant.py` - Test Qdrant connection
- `ingest_sample_content.py` - Ingest sample content
- `rag_tester.py` - CLI-based RAG testing
- `init_db.py` - Initialize database tables

## API Endpoints

- Health check: `GET http://localhost:8000/health`
- Chat: `POST http://localhost:8000/v1/chat/sessions/{session_id}/messages`
- Auth: `POST http://localhost:8000/v1/auth/register`
- Content: `GET http://localhost:8000/v1/content/modules`

## Frontend URLs

- Main site: `http://localhost:3000`
- Chat widget: Available on all pages (bottom right)
- Authentication: `/signin`, `/signup`, `/profile`

## Getting API Keys

1. **OpenRouter API Key**:
   - Visit [OpenRouter](https://openrouter.ai/)
   - Sign up for an account
   - Navigate to "API Keys" section
   - Create a new key and copy it

2. **Qdrant API Key** (if using cloud):
   - Visit [Qdrant Cloud](https://cloud.qdrant.io/)
   - Sign up for an account
   - Create a cluster and copy the API key

## Development Tips

- Always ensure Qdrant is running before starting the backend
- Use the sample content ingestion script to populate your vector database
- Check logs in the backend terminal for debugging information
- The system will still function without RAG (will use LLM without context), but with reduced functionality

## Support

If you encounter issues:
1. Check that all services (Qdrant, Backend, Frontend) are running
2. Verify all environment variables are set correctly
3. Review the logs for specific error messages
4. Consult the troubleshooting section above