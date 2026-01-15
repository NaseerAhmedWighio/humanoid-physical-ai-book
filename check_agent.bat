@echo off
REM Agent Diagnostic Script for Humanoid AI Book Backend
REM This script checks all dependencies and configurations for the chatbot

echo ========================================
echo Humanoid AI Book - Agent Diagnostic Tool
echo ========================================
echo.

REM Check if we're in the right directory
if not exist "backend" (
    echo Error: backend directory not found!
    echo Please run this script from the project root directory.
    pause
    exit /b 1
)

echo 1. Checking Python installation...
python --version
if %errorlevel% neq 0 (
    echo Error: Python is not installed or not in PATH
    pause
    exit /b 1
)
echo Python is available
echo.

echo 2. Checking required Python packages...
cd backend
python -c "import fastapi; print('FastAPI: OK')"
if %errorlevel% neq 0 (echo FastAPI: MISSING & pause & exit /b 1)

python -c "import openai; print('OpenAI: OK')"
if %errorlevel% neq 0 (echo OpenAI: MISSING & pause & exit /b 1)

python -c "import qdrant_client; print('Qdrant Client: OK')"
if %errorlevel% neq 0 (echo Qdrant Client: MISSING & pause & exit /b 1)

python -c "import fastembed; print('FastEmbed: OK')"
if %errorlevel% neq 0 (echo FastEmbed: MISSING & pause & exit /b 1)

python -c "import sqlalchemy; print('SQLAlchemy: OK')"
if %errorlevel% neq 0 (echo SQLAlchemy: MISSING & pause & exit /b 1)

echo All required packages are available
echo.

echo 3. Checking environment variables...
if not defined GEMINI_API_KEY (
    echo Warning: GEMINI_API_KEY environment variable not set
    echo Please set GEMINI_API_KEY with your actual API key
) else (
    echo GEMINI_API_KEY: SET
)

if not defined QDRANT_URL (
    echo Warning: QDRANT_URL environment variable not set
    echo Defaulting to http://localhost:6333
) else (
    echo QDRANT_URL: %QDRANT_URL%
)

echo.

echo 4. Testing application imports...
echo Testing main application...
python -c "from src.main import app; print('Main app: OK')"
if %errorlevel% neq 0 (echo Main app: ERROR & pause & exit /b 1)

echo Testing chat API...
python -c "from src.api.chat import router; print('Chat API: OK')"
if %errorlevel% neq 0 (echo Chat API: ERROR & pause & exit /b 1)

echo Testing LLM service...
python -c "from src.services.llm_service import llm_service; print('LLM Service: OK')"
if %errorlevel% neq 0 (echo LLM Service: ERROR & pause & exit /b 1)

echo Testing retrieving service...
python -c "from src.services.retrieving import retrieve; print('Retrieving Service: OK')"
if %errorlevel% neq 0 (echo Retrieving Service: ERROR & pause & exit /b 1)

echo.

echo 5. Testing model imports...
python -c "from src.models.chat_message import ChatMessage; print('ChatMessage model: OK')"
if %errorlevel% neq 0 (echo ChatMessage model: ERROR & pause & exit /b 1)

python -c "from src.models.chat_session import ChatSession; print('ChatSession model: OK')"
if %errorlevel% neq 0 (echo ChatSession model: ERROR & pause & exit /b 1)

echo.

echo 6. Checking for problematic imports...
echo Testing for any agents library imports that might cause issues...
python -c "try: import agents; print('Warning: agents library found but may cause issues'); except ImportError: print('agents library: Not found - Good')"
echo.

echo 7. Creating a test environment file if it doesn't exist...
if not exist ".env" (
    echo GEMINI_API_KEY=your-gemini-api-key-here > .env
    echo GEMINI_MODEL=gemini-2.0-flash >> .env
    echo LLM_PROVIDER=gemini >> .env
    echo QDRANT_URL=http://localhost:6333 >> .env
    echo QDRANT_API_KEY=your-qdrant-api-key-here >> .env
    echo DATABASE_URL=postgresql://username:password@localhost/dbname >> .env
    echo ALLOWED_ORIGINS=* >> .env
    echo Created .env file with default values
) else (
    echo .env file already exists
)

echo.

echo 8. Running a basic server test...
echo Attempting to start server in background...
start /min cmd /c "python -c \"from src.main import app; print('Server ready to start'); import time; time.sleep(2)\""
echo Server test completed
echo.

echo ========================================
echo DIAGNOSTIC COMPLETED
echo ========================================
echo.
echo If all tests passed above, your system should be ready.
echo.
echo To run the server:
echo 1. Set your GEMINI_API_KEY environment variable
echo 2. Make sure Qdrant is running on your specified URL
echo 3. Run: uvicorn src.main:app --reload
echo.
echo Press any key to exit...
pause >nul