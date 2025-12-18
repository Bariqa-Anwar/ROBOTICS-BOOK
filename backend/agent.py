import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import cohere
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load API Keys
load_dotenv()

app = FastAPI()

# Enable CORS so Docusaurus can talk to this backend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize Clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant = QdrantClient(path="qdrant_storage") 

class ChatRequest(BaseModel):
    message: str

@app.post("/chat")
async def chat(request: ChatRequest):
    try:
        # 1. Translate your text into a Vector
        # We use v3.0 here, which remains a standard search model in 2025
        embed_res = co.embed(
            texts=[request.message],
            model="embed-english-v3.0", 
            input_type="search_query"
        )
        query_vector = embed_res.embeddings[0]

        # 2. Search Qdrant for context
        search_result = qdrant.query_points(
            collection_name="robotics_book",
            query=query_vector, 
            limit=3
        ).points

        context = "\n".join([r.payload['text'] for r in search_result]) if search_result else ""

        # 3. Generate Response with an ACTIVE 2025 model
        # UPDATED: command-r-plus replaced with command-a-03-2025
        prompt = f"Context from the Robotics Book:\n{context}\n\nUser Question: {request.message}\nAnswer:"
        
        response = co.chat(
            message=prompt,
            model="command-r-08-2024" 
        )

        return {"response": response.text}
        
    except Exception as e:
        print(f"Backend Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)