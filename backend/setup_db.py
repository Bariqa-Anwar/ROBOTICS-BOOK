import os
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from dotenv import load_dotenv

load_dotenv()

# Initialize Qdrant
qdrant = QdrantClient(path="qdrant_storage")

def initialize_database():
    collection_name = "robotics_book"

    # 1. Safely create the collection
    print(f"Checking if collection '{collection_name}' exists...")
    if qdrant.collection_exists(collection_name):
       qdrant.delete_collection(collection_name)
    
    qdrant.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
    )

    # 2. Add initial book context using PointStruct (the fix for your error)
    print("Adding initial book context...")
    qdrant.upsert(
        collection_name=collection_name,
        points=[
            PointStruct(
                id=1,
                vector=[0.1] * 1024, # Dummy vector for now
                payload={"text": "This book covers ROS 2, Isaac Sim, and Humanoid Robotics development for the decade 2025-2035."}
            )
        ]
    )
    print("✅ Database initialized successfully!")

    # ADD THIS LINE TO FIX THE SHUTDOWN ERROR:
    qdrant.close()

if __name__ == "__main__":
    initialize_database()