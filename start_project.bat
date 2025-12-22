@echo off
echo Starting Physical AI and Humanoid Robotics Textbook Project with Authentication and Personalization...

REM Start Redis (make sure it's installed and in PATH)
echo Starting Redis server...
start /min redis-server

REM Wait a moment for Redis to start
timeout /t 3 /nobreak >nul

REM Start FastAPI Backend
echo Starting FastAPI backend server...
cd /d "E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook\fastapi_app"
start /min cmd /c "python -m uvicorn app:app --host 0.0.0.0 --port 8000"
REM Alternative if uvicorn is not in PATH: start /min cmd /c "python -c \"import uvicorn; uvicorn.run('app:app', host='0.0.0.0', port=8000)\""

REM Wait for backend to start
timeout /t 5 /nobreak >nul

REM Start Docusaurus Book (Frontend) - this serves both the book and our new auth/personalization features
echo Starting Docusaurus book with authentication and personalization features...
cd /d "E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook"
start cmd /c "npm run start"

echo.
echo All services started successfully!
echo.
echo Backend (FastAPI): http://localhost:8000
echo Book with Auth & Personalization: http://localhost:3002
echo.
echo The project includes:
echo - Authentication with Better Auth
echo - Chapter personalization based on user background
echo - Personalization state persistence
echo - User metadata collection (skill_level, hardware_background, learning_goal)
echo.
echo Press any key to exit...
pause >nul