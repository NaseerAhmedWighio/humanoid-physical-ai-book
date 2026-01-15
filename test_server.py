import requests
import json
import time
import threading

def test_server():
    # Wait a moment for server to be ready
    time.sleep(2)

    try:
        # Test the root endpoint
        print("Testing root endpoint...")
        response = requests.get("http://127.0.0.1:8000/")
        print(f"Root endpoint: Status {response.status_code}")
        if response.status_code != 200:
            print(f"Root endpoint error: {response.text}")

        # Test health endpoint
        print("Testing health endpoint...")
        response = requests.get("http://127.0.0.1:8000/health")
        print(f"Health endpoint: Status {response.status_code}")
        if response.status_code != 200:
            print(f"Health endpoint error: {response.text}")

        # Test creating a chat session
        print("Testing chat session creation...")
        session_data = {"title": "Test Session"}
        response = requests.post("http://127.0.0.1:8000/v1/chat/sessions", json=session_data)
        print(f"Session creation: Status {response.status_code}")
        if response.status_code != 200:
            print(f"Session creation error: {response.text}")
        else:
            # Get the session ID from response
            session_response = response.json()
            session_id = session_response.get('session_id', 'test_session')

            # Test sending a message to the chat endpoint
            print(f"Testing chat message to session {session_id}...")
            message_data = {"content": "Hello", "context_window": 1}
            response = requests.post(f"http://127.0.0.1:8000/v1/chat/sessions/{session_id}/messages", json=message_data)
            print(f"Message sending: Status {response.status_code}")
            if response.status_code != 200:
                print(f"Message sending error: {response.text}")

    except requests.exceptions.ConnectionError:
        print("Could not connect to server - make sure it's running on port 8000")
    except Exception as e:
        print(f"Test error: {e}")

if __name__ == "__main__":
    test_server()