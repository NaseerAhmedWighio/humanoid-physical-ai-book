# Environment Configuration Guide

This guide explains how to set up the environment variables for the Physical AI & Humanoid Robotics Textbook application.

## Backend Environment Variables

### Required Variables

1. **Database Configuration**
   - `DATABASE_URL`: PostgreSQL database connection string
     - Format: `postgresql://username:password@host:port/database_name`
     - Example: `postgresql://myuser:mypassword@localhost:5432/humanoid_ai_textbook`
   - `NEON_DATABASE_URL`: Neon PostgreSQL database URL (if using Neon)

2. **LLM Provider Configuration**
   - `LLM_PROVIDER`: LLM provider to use (default: `gemini`, options: `openai`, `gemini`)

   For Gemini (recommended):
   - `GEMINI_API_KEY`: Your Google Gemini API key
     - Get it from: https://aistudio.google.com/
   - `GEMINI_MODEL`: Gemini model to use (default: `gemini-1.5-pro`)
   - `GEMINI_EMBEDDING_MODEL`: Gemini embedding model (default: `embedding-001`)

   For OpenAI:
   - `OPENAI_API_KEY`: Your OpenAI API key
     - Get it from: https://platform.openai.com/api-keys
   - `OPENAI_MODEL`: OpenAI model to use (default: `gpt-4-turbo`)
   - `EMBEDDING_MODEL`: OpenAI embedding model (default: `text-embedding-3-small`)

3. **Qdrant Configuration**
   - `QDRANT_URL`: Qdrant vector database URL
     - Local: `http://localhost:6333`
     - Cloud: `https://your-cluster-url.qdrant.io`
   - `QDRANT_API_KEY`: Qdrant API key (if using cloud service)

### Optional Variables

4. **Application Configuration**
   - `APP_ENV`: Environment (development, staging, production)
   - `DEBUG`: Enable debug mode (true/false)

5. **CORS Configuration**
   - `FRONTEND_URL`: URL of your frontend application
   - `ALLOWED_ORIGINS`: Comma-separated list of allowed origins

6. **Server Configuration**
   - `HOST`: Server host (default: 0.0.0.0)
   - `PORT`: Server port (default: 8000)

7. **JWT Configuration**
   - `JWT_SECRET_KEY`: Secret key for JWT tokens (use a strong random string)
   - `JWT_ALGORITHM`: Algorithm for JWT (default: HS256)
   - `ACCESS_TOKEN_EXPIRE_MINUTES`: Token expiration time

## Frontend Environment Variables

### Required Variables

1. **Backend API Configuration**
   - `API_BASE_URL`: Base URL of your backend API
     - Development: `http://localhost:8000`
     - Production: Your deployed backend URL

## Setup Instructions

### 1. Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Copy the example environment file:
   ```bash
   cp .env.example .env
   ```

3. Edit the `.env` file and add your values:
   ```bash
   # Open the file in your preferred editor
   nano .env  # or use your preferred editor
   ```

4. Add your API keys and configuration:
   - Set your OpenAI API key
   - Configure your database connection
   - Set up Qdrant connection

### 2. Frontend Setup

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Copy the example environment file:
   ```bash
   cp .env.example .env
   ```

3. Edit the `.env` file and add your values:
   ```bash
   # Open the file in your preferred editor
   nano .env  # or use your preferred editor
   ```

## Required Services to Set Up

### 1. LLM Provider Setup

Option 1: Google Gemini (Recommended)
- Sign up at: https://aistudio.google.com/
- Navigate to: Get API Key section
- Create a new API key
- Add it to your backend `.env` file as `GEMINI_API_KEY`
- Set `LLM_PROVIDER=gemini` in your `.env` file

Option 2: OpenAI
- Sign up at: https://platform.openai.com/
- Navigate to: Settings > API Keys
- Create a new secret key
- Add it to your backend `.env` file as `OPENAI_API_KEY`
- Set `LLM_PROVIDER=openai` in your `.env` file

### 2. Qdrant Vector Database
Option 1: Local Installation (Recommended for development)
- Install Docker: https://docs.docker.com/get-docker/
- Run Qdrant locally:
  ```bash
  docker run -p 6333:6333 -p 6334:6334 \
    -v $(pwd)/qdrant_storage:/qdrant/storage:z \
    qdrant/qdrant
  ```
- Set `QDRANT_URL=http://localhost:6333` in your `.env`

Option 2: Cloud Qdrant
- Sign up at: https://cloud.qdrant.io/
- Create a cluster
- Get the cluster URL and API key
- Set `QDRANT_URL` and `QDRANT_API_KEY` in your `.env`

### 3. PostgreSQL Database
Option 1: Local Installation
- Install PostgreSQL: https://www.postgresql.org/download/
- Create a database for the application
- Set `DATABASE_URL` in your `.env`

Option 2: Neon (Recommended)
- Sign up at: https://neon.tech/
- Create a new project
- Get the connection string
- Set `NEON_DATABASE_URL` in your `.env`

## Security Best Practices

1. **Never commit .env files** - They are already in `.gitignore`
2. **Use strong JWT secrets** - Generate a random 32+ character string
3. **Rotate API keys regularly** - Especially in production
4. **Use different keys for different environments**
5. **Restrict database access** - Use strong passwords and limit permissions

## Troubleshooting

### Common Issues

1. **Environment variables not loading**
   - Make sure your `.env` file is in the correct directory
   - Check that the file is named exactly `.env` (not `.env.txt`)
   - Restart your application after changing environment variables

2. **OpenAI API errors**
   - Verify your API key is correct
   - Check that your OpenAI account is properly set up
   - Ensure you have sufficient credits in your OpenAI account

3. **Database connection errors**
   - Verify your database URL is correct
   - Check that your database server is running
   - Ensure your database credentials are correct

4. **Qdrant connection errors**
   - Verify your Qdrant URL is correct
   - Check that your Qdrant server is running
   - If using cloud Qdrant, verify your API key

## Example .env Files

### Backend .env Example

For Google Gemini (Recommended):
```env
DATABASE_URL=postgresql://myuser:mypassword@localhost:5432/humanoid_ai_textbook
NEON_DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/mydb
LLM_PROVIDER=gemini
GEMINI_API_KEY=your-gemini-api-key-here
GEMINI_MODEL=gemini-1.5-pro
GEMINI_EMBEDDING_MODEL=embedding-001
QDRANT_URL=http://localhost:6333
JWT_SECRET_KEY=very-long-random-string-with-at-least-32-characters
```

For OpenAI:
```env
DATABASE_URL=postgresql://myuser:mypassword@localhost:5432/humanoid_ai_textbook
NEON_DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/mydb
LLM_PROVIDER=openai
OPENAI_API_KEY=sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
OPENAI_MODEL=gpt-4-turbo
EMBEDDING_MODEL=text-embedding-3-small
QDRANT_URL=http://localhost:6333
JWT_SECRET_KEY=very-long-random-string-with-at-least-32-characters
```

### Frontend .env Example
```env
API_BASE_URL=http://localhost:8000
NODE_ENV=development
```

## Testing Your Configuration

1. Start your backend:
   ```bash
   cd backend
   uvicorn src.main:app --reload
   ```

2. Verify your API is running by visiting:
   http://localhost:8000/health

3. Check that environment variables are loaded by looking at the startup logs.

4. Index your content to test the RAG system:
   ```bash
   cd backend
   python scripts/index_content.py
   ```

If you encounter any issues with your configuration, check the application logs for specific error messages and verify that all required environment variables are properly set.