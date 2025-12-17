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
qdrant = QdrantClient(path="qdrant_storage") # Or your specific URL/Path

class ChatRequest(BaseModel):
    message: str

@app.post("/chat")
async def chat(request: ChatRequest):
    try:
        # 1. Search Qdrant for book context
        search_result = qdrant.query_points(
            collection_name="robotics_book",
            query_content=request.message,
            limit=3
        ).points

        context = "\n".join([r.payload['text'] for r in search_result])

        # 2. Generate Response with Cohere
        prompt = f"Context from the Robotics Book:\n{context}\n\nUser Question: {request.message}\nAnswer:"
        
        response = co.chat(
            message=prompt,
            model="command-r-plus"
        )

        return {"reply": response.text}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)