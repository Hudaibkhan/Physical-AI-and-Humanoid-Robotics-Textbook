@echo off
echo Starting RAG Chatbot Project...

REM Start Redis (make sure it's installed and in PATH)
echo Starting Redis server...
start /min redis-server

REM Wait a moment for Redis to start
timeout /t 3 /nobreak >nul

REM Start Backend
echo Starting backend server...
cd /d "E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook\backend"
start /min cmd /c "venv\Scripts\activate && uvicorn src.main:app --host 0.0.0.0 --port 8000"

REM Wait for backend to start
timeout /t 5 /nobreak >nul

REM Start Frontend
echo Starting frontend server...
cd /d "E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook\frontend"
start /min cmd /c "npm start"

REM Wait for frontend to start
timeout /t 5 /nobreak >nul

REM Start Docusaurus Book
echo Starting Docusaurus book...
cd /d "E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook"
start cmd /c "npm run start"

echo All services started successfully!
echo.
echo Backend: http://localhost:8000
echo Frontend: http://localhost:3000
echo Book: http://localhost:3000 (or another port if taken)
echo.
echo Press any key to exit...
pause >nul