import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from uuid import UUID
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_client = QdrantClient(url=self.qdrant_url)

        # Define collection name for content chunks
        self.collection_name = "content_chunks"

        # Initialize the collection if it doesn't exist
        self._init_collection()

    def _init_collection(self):
        """Initialize the Qdrant collection for content chunks"""
        try:
            # Check if collection exists
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection with vector configuration
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,  # Standard size for OpenAI embeddings
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {e}")
            raise

    def add_content_chunk(self, chunk_id: str, content: str, embedding: List[float],
                         module_id: str, metadata: Optional[Dict[str, Any]] = None):
        """Add a content chunk to the vector database"""
        try:
            if metadata is None:
                metadata = {}

            metadata.update({
                "module_id": module_id,
                "content": content  # Store the content for retrieval
            })

            points = [PointStruct(
                id=chunk_id,
                vector=embedding,
                payload=metadata
            )]

            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Added content chunk {chunk_id} to Qdrant")
            return True
        except Exception as e:
            logger.error(f"Error adding content chunk to Qdrant: {e}")
            return False

    def search_content(self, query_embedding: List[float], limit: int = 10) -> List[Dict[str, Any]]:
        """Search for relevant content using vector similarity"""
        try:
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )

            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "module_id": result.payload.get("module_id"),
                    "score": result.score,
                    "metadata": {k: v for k, v in result.payload.items()
                               if k not in ["content", "module_id"]}
                })

            logger.info(f"Found {len(results)} results for search query")
            return results
        except Exception as e:
            logger.error(f"Error searching content in Qdrant: {e}")
            return []

    def delete_content_chunk(self, chunk_id: str) -> bool:
        """Delete a content chunk from the vector database"""
        try:
            self.qdrant_client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[chunk_id]
                )
            )
            logger.info(f"Deleted content chunk {chunk_id} from Qdrant")
            return True
        except Exception as e:
            logger.error(f"Error deleting content chunk from Qdrant: {e}")
            return False

    def get_content_chunk(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """Retrieve a specific content chunk by ID"""
        try:
            records = self.qdrant_client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id]
            )

            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "content": record.payload.get("content", ""),
                    "module_id": record.payload.get("module_id"),
                    "metadata": {k: v for k, v in record.payload.items()
                               if k not in ["content", "module_id"]}
                }
            return None
        except Exception as e:
            logger.error(f"Error retrieving content chunk from Qdrant: {e}")
            return None

# Global instance of RAGService
rag_service = RAGService()