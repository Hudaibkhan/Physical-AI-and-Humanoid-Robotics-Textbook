# Start RAG Chatbot Project

Write-Host "Starting RAG Chatbot Project..." -ForegroundColor Green

# Start Redis (make sure it's installed and in PATH)
Write-Host "Starting Redis server..." -ForegroundColor Yellow
Start-Process -FilePath "redis-server" -WindowStyle Minimized

# Wait a moment for Redis to start
Start-Sleep -Seconds 3

# Start Backend
Write-Host "Starting backend server..." -ForegroundColor Yellow
Set-Location "E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook\backend"
Start-Process -FilePath "cmd" -ArgumentList "/c", "venv\Scripts\activate && uvicorn src.main:app --host 0.0.0.0 --port 8000" -WindowStyle Minimized

# Wait for backend to start
Start-Sleep -Seconds 5

# Start Frontend
Write-Host "Starting frontend server..." -ForegroundColor Yellow
Set-Location "E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook\frontend"
Start-Process -FilePath "cmd" -ArgumentList "/c", "npm start" -WindowStyle Minimized

# Wait for frontend to start
Start-Sleep -Seconds 5

# Start Docusaurus Book
Write-Host "Starting Docusaurus book..." -ForegroundColor Yellow
Set-Location "E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook"
Start-Process -FilePath "cmd" -ArgumentList "/c", "npm run start"

Write-Host "All services started successfully!" -ForegroundColor Green
Write-Host ""
Write-Host "Backend: http://localhost:8000" -ForegroundColor Cyan
Write-Host "Frontend: http://localhost:3000" -ForegroundColor Cyan
Write-Host "Book: http://localhost:3000 (or another port if taken)" -ForegroundColor Cyan
Write-Host ""