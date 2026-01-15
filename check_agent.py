import os
import sys
import traceback
from pathlib import Path

def check_agent_issues():
    """Comprehensive diagnostic for agent-related issues in the humanoid AI book backend"""

    print("=" * 60)
    print("Humanoid AI Book - Agent Diagnostic Tool")
    print("=" * 60)

    project_root = Path(__file__).parent
    backend_path = project_root / "backend"

    if not backend_path.exists():
        print("ERROR: backend directory not found!")
        print("Please run this script from the project root directory.")
        return False

    print(f"Project root: {project_root}")
    print(f"Backend path: {backend_path}")
    print()

    # Change to backend directory
    os.chdir(backend_path)
    sys.path.insert(0, str(backend_path))

    # Check 1: Environment variables
    print("1. Checking environment variables...")
    required_vars = ['OPENROUTER_API_KEY', 'QDRANT_URL']
    for var in required_vars:
        if not os.getenv(var):
            print(f"   WARNING: {var} not set - using default")
            if var == 'OPENROUTER_API_KEY':
                os.environ[var] = 'dummy-key-for-test'
            elif var == 'QDRANT_URL':
                os.environ[var] = 'http://localhost:6333'
        else:
            print(f"   {var}: SET")
    print()

    # Check 2: Import issues
    print("2. Testing imports...")

    # Test core imports
    test_imports = [
        ("FastAPI", "fastapi"),
        ("OpenAI", "openai"),
        ("Qdrant Client", "qdrant_client"),
        ("FastEmbed", "fastembed"),
        ("SQLAlchemy", "sqlalchemy"),
        ("Python-dotenv", "dotenv")
    ]

    for name, module in test_imports:
        try:
            __import__(module)
            print(f"   OK {name}: Available")
        except ImportError as e:
            print(f"   ERROR {name}: Missing - {e}")

    print()

    # Check 3: Specific file imports that might have agent issues
    print("3. Testing problematic imports...")

    problematic_imports = [
        ("src.services.llm_service", "LLM Service"),
        ("src.services.agent", "Agent Service"),
        ("src.api.chat", "Chat API"),
        ("src.models.chat_message", "Chat Message Model"),
        ("src.models.chat_session", "Chat Session Model"),
        ("src.services.retrieving", "Retrieving Service")
    ]

    for module_path, description in problematic_imports:
        try:
            # Import the module
            if '.' in module_path:
                parts = module_path.split('.')
                module = __import__(parts[0])
                for part in parts[1:]:
                    module = getattr(module, part)
            else:
                module = __import__(module_path)
            print(f"   OK {description}: Imported successfully")
        except Exception as e:
            print(f"   ERROR {description}: Error - {e}")
            print(f"     Traceback: {traceback.format_exc()}")

    print()

    # Check 4: Check for any remaining agents library references
    print("4. Checking for agents library references...")

    files_to_check = [
        "src/services/llm_service.py",
        "src/services/agent.py",
        "src/api/chat.py",
        "src/main.py"
    ]

    for file_path in files_to_check:
        full_path = backend_path / file_path
        if full_path.exists():
            with open(full_path, 'r', encoding='utf-8') as f:
                content = f.read()
                if 'agents' in content:
                    print(f"   WARNING Found 'agents' reference in {file_path}")
                    # Find the specific lines
                    lines = content.split('\n')
                    for i, line in enumerate(lines, 1):
                        if 'agents' in line.lower():
                            print(f"     Line {i}: {line.strip()}")
                else:
                    print(f"   OK No 'agents' reference in {file_path}")
        else:
            print(f"   - File {file_path} does not exist")

    print()

    # Check 5: Test the LLM service specifically
    print("5. Testing LLM service initialization...")
    try:
        from src.services.llm_service import llm_service
        print("   OK LLM Service initialized successfully")
    except Exception as e:
        print(f"   ERROR LLM Service initialization failed: {e}")
        print(f"     Traceback: {traceback.format_exc()}")

    print()

    # Check 6: Test chat functionality
    print("6. Testing chat functionality...")
    try:
        from src.api.chat import ChatMessageRequest
        print("   OK ChatMessageRequest class loaded")

        # Create a test request
        test_request = ChatMessageRequest(content="Hello", context_window=1)
        print("   OK ChatMessageRequest instance created")
    except Exception as e:
        print(f"   ERROR Chat functionality error: {e}")
        print(f"     Traceback: {traceback.format_exc()}")

    print()

    # Check 7: Check models for import errors
    print("7. Testing model imports...")
    try:
        from src.models.chat_message import ChatMessage
        print("   OK ChatMessage model imported")
    except Exception as e:
        print(f"   ERROR ChatMessage model error: {e}")
        print(f"     Traceback: {traceback.format_exc()}")

    try:
        from src.models.chat_session import ChatSession
        print("   OK ChatSession model imported")
    except Exception as e:
        print(f"   ERROR ChatSession model error: {e}")
        print(f"     Traceback: {traceback.format_exc()}")

    print()

    # Check 8: Test main app
    print("8. Testing main application...")
    try:
        from src.main import app
        print("   OK Main application imported successfully")
    except Exception as e:
        print(f"   ERROR Main application error: {e}")
        print(f"     Traceback: {traceback.format_exc()}")

    print()
    print("=" * 60)
    print("DIAGNOSTIC COMPLETED")
    print("=" * 60)

    print("\nNext steps if issues found:")
    print("1. Make sure all required packages are installed: pip install -r requirements.txt")
    print("2. Set required environment variables (OPENROUTER_API_KEY, QDRANT_URL)")
    print("3. Check that Qdrant is running if using RAG functionality")
    print("4. Verify that your API keys are valid and have proper permissions")

    return True

if __name__ == "__main__":
    check_agent_issues()