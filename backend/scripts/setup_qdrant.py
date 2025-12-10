"""
Script to set up Qdrant vector database for the Humanoid AI Textbook RAG functionality
"""
import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import VectorParams, Distance
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def setup_qdrant():
    """Set up Qdrant collections and configurations"""
    print("Setting up Qdrant vector database...")

    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
    print(f"Connecting to Qdrant at: {qdrant_url}")

    client = QdrantClient(url=qdrant_url)

    # Define collection name for content chunks
    collection_name = "content_chunks"

    try:
        # Check if collection exists
        collections = client.get_collections()
        collection_exists = any(col.name == collection_name for col in collections.collections)

        if not collection_exists:
            # Create collection with vector configuration
            client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=1536,  # Standard size for OpenAI embeddings
                    distance=Distance.COSINE
                )
            )
            print(f"✓ Created Qdrant collection: {collection_name}")
        else:
            print(f"✓ Qdrant collection {collection_name} already exists")

        # Create another collection for exercises if needed
        exercises_collection = "exercise_chunks"
        exercises_exists = any(col.name == exercises_collection for col in collections.collections)

        if not exercises_exists:
            client.create_collection(
                collection_name=exercises_collection,
                vectors_config=VectorParams(
                    size=1536,
                    distance=Distance.COSINE
                )
            )
            print(f"✓ Created Qdrant collection: {exercises_collection}")
        else:
            print(f"✓ Qdrant collection {exercises_collection} already exists")

        print("✓ Qdrant setup completed successfully!")

        # Print collection info
        collections = client.get_collections()
        print("\nCurrent collections:")
        for col in collections.collections:
            info = client.get_collection(col.name)
            print(f"  - {col.name}: {info.points_count} vectors")

    except Exception as e:
        print(f"✗ Error setting up Qdrant: {e}")
        return False

    return True

if __name__ == "__main__":
    success = setup_qdrant()
    if success:
        print("\nQdrant is now ready for RAG functionality!")
    else:
        print("\nQdrant setup failed!")
        exit(1)