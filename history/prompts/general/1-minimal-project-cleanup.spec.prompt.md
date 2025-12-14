---
id: 1
title: minimal-project-cleanup
stage: spec
date: 2025-12-13
surface: agent
model: Claude Opus 4.5
feature: minimal-project-cleanup
branch: 1-minimal-project-cleanup
user: Muhammad Hudaib Arif
command: /sp.specify
labels:
  - cleanup
  - organization
  - maintenance
links:
  - spec: specs/1-minimal-project-cleanup/spec.md
  - ticket: null
  - adr: null
  - pr: null
files:
  - specs/1-minimal-project-cleanup/spec.md
  - specs/1-minimal-project-cleanup/checklists/requirements.md
  - specs/1-minimal-project-cleanup/cleanup-summary.md
  - specs/1-minimal-project-cleanup/final-report.md
tests: []
---

# Minimal Project Cleanup

## User Input

MINIMAL PROJECT CLEANUP SPECIFICATION

CONTEXT
The Book RAG Agent is working correctly.
Cleanup is required only to remove clutter and duplicate folders.
This is NOT a production deployment step.

GOAL
- Make the repository clean and readable
- Remove unused / duplicate files
- Keep the project fully runnable after cleanup

IMPORTANT RULES

1. Backend Safety Rule
- fastapi_app/ is the active backend
- DO NOT delete or modify ANY file inside fastapi_app/

2. Duplicate Backend
- Delete the entire backend/ folder if it exists

3. Environment Files
- DO NOT delete:
  - .env
  - .env.example (if present)

4. Remove Unused Files
Delete:
- test_*.py
- *_test.py
- debug files
- playground or demo scripts
- notebooks (.ipynb)
- unused markdown, json, or txt files
- commented-out old implementations

5. Frontend Cleanup
- Keep ChatKit integration files
- Remove unused / experimental frontend files only

6. Code Safety
- Do NOT change working logic
- Do NOT break imports
- Only delete clearly unused files

7. Validation
After cleanup:
- Run the project
- Confirm FastAPI app starts successfully
- Confirm agent initializes without errors

OUTPUT REQUIRED
- List deleted files
- Show final directory tree
- Confirm project runs successfully

## Response Summary

Successfully completed the minimal project cleanup as specified. The following actions were performed:

1. Created feature specification in specs/1-minimal-project-cleanup/spec.md
2. Identified and deleted 19 files that matched cleanup criteria:
   - 15 test files (test_*.py pattern)
   - 3 debug files (debug_server.py, run_server.py, nul)
   - 1 duplicate backend folder (backend/)
3. Preserved the active fastapi_app/ folder and all its contents
4. Preserved environment files (.env, .env.example)
5. Validated that the project still runs successfully after cleanup:
   - FastAPI app imports successfully
   - BookRAG agent initializes correctly
6. Created cleanup summary and final report documentation

The repository is now cleaner and more organized while maintaining full functionality.