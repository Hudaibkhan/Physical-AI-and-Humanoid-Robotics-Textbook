# Quick Start: Running the RAG Chatbot Project with Your Credentials

## Your Credentials Have Been Configured

Your Qdrant and Gemini credentials have been automatically configured in the project:

- Qdrant URL: https://88b96840-d5e8-477c-9fd2-4fbd361591ee.us-east4-0.gcp.cloud.qdrant.io
- Qdrant API Key: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.5ezXFLCiBuQxCu7f50ZHOMmMO6HbtX7dTG75KTuVrSY
- Gemini API Key: AIzaSyDCiLd0McLcljmMcOegzuZznxFOD_WuRhc

## Steps to Run the Project

### 1. Make sure Redis is running
- Install Redis if not already installed
- Start Redis server (typically runs on localhost:6379)

### 2. Upload your book content to Qdrant
Open a command prompt/terminal and run:

```bash
cd E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook\backend
# Activate virtual environment
venv\Scripts\activate
# Run the upload script
python scripts/upload_chunks.py
```

### 3. Start all services
You can either:

**Option A: Use the start script (Windows)**
```bash
# From the project root directory
start_all.bat
```

**Option B: Start services manually**
- Start backend: `cd backend && venv\Scripts\activate && uvicorn src.main:app --host 0.0.0.0 --port 8000`
- Start frontend: `cd frontend && npm start`
- Start book: `cd . && npm run start`

### 4. Access the application
- Book/Documentation: http://localhost:3000
- Backend API: http://localhost:8000
- Chatbot widget will appear on all book pages

## Important Notes
- The upload script will process all .md files in the docs/ directory
- The chatbot will only answer questions based on your book content
- Out-of-book questions will be rejected with "This information is not in the book"
- Text highlighting functionality is available on all pages

## Troubleshooting
- If you get API quota errors, check your Gemini API key usage
- If Qdrant connection fails, verify your credentials are correct
- Make sure all services are running before testing the chatbot