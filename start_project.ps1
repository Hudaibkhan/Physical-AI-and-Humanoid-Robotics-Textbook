# Start Physical AI and Humanoid Robotics Textbook Project with Authentication and Personalization

Write-Host "Starting Physical AI and Humanoid Robotics Textbook Project with Authentication and Personalization..." -ForegroundColor Green

# Start Redis (make sure it's installed and in PATH)
Write-Host "Starting Redis server..." -ForegroundColor Yellow
Start-Process -FilePath "redis-server" -WindowStyle Minimized

# Wait a moment for Redis to start
Start-Sleep -Seconds 3

# Start FastAPI Backend
Write-Host "Starting FastAPI backend server..." -ForegroundColor Yellow
Set-Location "E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook\fastapi_app"
Start-Process -FilePath "cmd" -ArgumentList "/c", "python -m uvicorn app:app --host 0.0.0.0 --port 8000" -WindowStyle Minimized

# Wait for backend to start
Start-Sleep -Seconds 5

# Start Docusaurus Book (Frontend) - this serves both the book and our new auth/personalization features
Write-Host "Starting Docusaurus book with authentication and personalization features..." -ForegroundColor Yellow
Set-Location "E:\Q4_Officail\hackathon_01\physical-ai-and-humanoid-robotics-textbook"
Start-Process -FilePath "cmd" -ArgumentList "/c", "npm run start"

Write-Host ""
Write-Host "All services started successfully!" -ForegroundColor Green
Write-Host ""
Write-Host "Backend (FastAPI): http://localhost:8000" -ForegroundColor Cyan
Write-Host "Book with Auth & Personalization: http://localhost:3002" -ForegroundColor Cyan
Write-Host ""
Write-Host "The project includes:" -ForegroundColor White
Write-Host " - Authentication with Better Auth" -ForegroundColor White
Write-Host " - Chapter personalization based on user background" -ForegroundColor White
Write-Host " - Personalization state persistence" -ForegroundColor White
Write-Host " - User metadata collection (skill_level, hardware_background, learning_goal)" -ForegroundColor White
Write-Host ""