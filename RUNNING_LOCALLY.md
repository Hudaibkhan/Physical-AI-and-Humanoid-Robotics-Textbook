# Running the RAG Chatbot Project Locally

This guide will help you run the complete RAG Chatbot project locally with your credentials.

## Prerequisites

- Python 3.11+ installed
- Node.js 18+ installed
- Redis server running locally (for caching)
- Git installed

## Step 1: Install Redis

You'll need Redis running locally for caching functionality:

### Windows:
1. Download Redis from https://github.com/redis-windows/redis-windows
2. Or install via Chocolatey: `choco install redis-64`
3. Start Redis server: `redis-server`

### macOS:
1. Install via Homebrew: `brew install redis`
2. Start Redis server: `brew services start redis`

### Linux:
1. Install via package manager: `sudo apt-get install redis-server`
2. Start Redis server: `sudo systemctl start redis`

## Step 2: Backend Setup

1. Navigate to the backend directory:
```bash
cd E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook\backend
```

2. Create a virtual environment and install dependencies:
```bash
python -m venv venv
venv\Scripts\activate  # On Windows
# source venv/bin/activate  # On macOS/Linux
pip install -r requirements.txt
```

3. Your credentials have already been configured in the settings and .env file.

## Step 3: Upload Book Content to Qdrant

1. Before running the backend, you need to upload your book content to Qdrant:
```bash
python scripts/upload_chunks.py
```

This script will:
- Read your book content (from docs directory or specified path)
- Chunk the content into manageable pieces
- Generate embeddings using Gemini
- Upload the embeddings to your Qdrant cloud instance

## Step 4: Run the Backend

1. Start the backend server:
```bash
cd E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook\backend
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

The backend will be available at `http://localhost:8000`

## Step 5: Frontend Setup

1. Navigate to the frontend directory:
```bash
cd E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook\frontend
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

The frontend will be available at `http://localhost:3000`

## Step 6: Running with Docusaurus Book

If you're using Docusaurus for your book:

1. Navigate to the root directory:
```bash
cd E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook
```

2. Install Docusaurus dependencies:
```bash
npm install
```

3. Start the Docusaurus server:
```bash
npm run start
```

The book will be available at `http://localhost:3000` (or another port if 3000 is taken)

## Step 7: Alternative - Run All Services with Docker

If you prefer using Docker, you can use the provided Dockerfile and docker-compose:

1. Make sure Docker is installed and running
2. Create a docker-compose.yml file with your configurations
3. Run: `docker-compose up --build`

## Testing the Integration

1. Make sure the backend is running on `http://localhost:8000`
2. The chatbot widget should appear on your book pages
3. Test by asking questions about your book content
4. Test the text highlighting functionality
5. Verify that out-of-book questions return "This information is not in the book"

## Troubleshooting

1. **Connection Issues**: Make sure your Qdrant Cloud instance is accessible and credentials are correct
2. **Gemini API Issues**: Verify your Gemini API key is valid and has sufficient quota
3. **CORS Issues**: Ensure the backend allows requests from your frontend origin
4. **Redis Issues**: Make sure Redis server is running locally

## Environment Variables Reference

The following environment variables are used:

Backend (.env file):
- QDRANT_URL: Your Qdrant Cloud URL
- QDRANT_API_KEY: Your Qdrant API key
- GEMINI_API_KEY: Your Google Gemini API key
- GEMINI_MODEL: Gemini model to use (default: gemini-pro)
- EMBEDDING_MODEL: Embedding model to use (default: gemini-embedding-001)
- REDIS_HOST, REDIS_PORT: Redis connection settings
- DEBUG: Enable/disable debug mode

Frontend (.env file):
- REACT_APP_BACKEND_URL: Backend API URL