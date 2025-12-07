---
id: 0002
title: Update tasks for textbook update
stage: tasks
date: 2025-12-06
surface: agent
model: gemini-2.5-flash
feature: book-content-update
branch: 001-book-content-update
user: unknown
command: /sp.tasks
labels: ["tasks", "feature-update", "docusaurus"]
links:
  spec: specs/001-book-content-update/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/001-book-content-update/tasks.md
tests:

---

## Prompt

# Objective
Update and improve the existing Physical AI & Humanoid Robotics Textbook project stored inside the `docs/` directory without deleting or recreating existing modules. Only review, refine, and extend the existing content.

# Tasks

## 1. Sidebar Structure Update
- Update Docusaurus sidebar to match this exact structure:
 d` from scratch.
- Include:
  - Book cover (front page image)
  - Title + subtitle
  - 1–2 paragraph description of the textbook purpose
  - A grid of module cards:
    - Clicking each card jumps dire - Physical AI & Humanoid Robotics Textbook
    - Module 1: ROS 2 Nervous System
    - Module 2: Digital Twin Simulation
    - Module 3: AI Brain (NVIDIA Isaac)
    - Module 4: Vision-Language-Action Robotics
    - Capstone Project
    - Additional Materials
    - Weekly Roadmap
- Remove duplicated or unstructured sidebar entries.
- Ensure ordering is fixed consistently across all sidebars.

## 2. Header & Footer Enhancements
- Add company/logo image in the header.
- Add footer with links to **all modules** and external resources.

## 3. Review & Improve Existing `/docs/` Content
- Open each module and submodule file inside `docs/`.
- Do NOT delete existing work.
- Improve clarity, flow, and formatting.
- Convert long text into clean sections, bullet points, callouts where helpful.
- Fix any Markdown formatting issues.

## 4. Add Missing Content in Incomplete Files
- Identify files that currently have little or no content (e.g., `Additional Materials`, `Weekly Roadmap`, some module intros).
- Write appropriatctly to that module

## 7. Apply a Consistent Theme Across the Book
- Set a **single unified color theme** for the textbook.
- Apply theme to:
  - Callouts
  - Headings
  - Diagrams
  - Cards
  - Buttons (if used)
- Ensure aesthetic consistency in the Markdown and Docusaurus config.

## 8. Final QA Review
- Ensure no broken links.
- Ensure all modules are reachable.
- Re-check sidebar navigation flow.
- Ensure all images load correctly.
- Verify consistency in titles, typography, spacing, and formatting.

## Response snapshot

I have successfully updated the `tasks.md` file based on your latest input. The task list is now structured into eight distinct phases, covering all the objectives you outlined for the textbook update. Each task has a unique ID and specifies the relevant file path for implementation.

The updated task list is located at `specs/001-book-content-update/tasks.md`.

## Outcome

- ✅ Impact: Successfully updated the tasks.md for the textbook update with a new, detailed task list.
- 🧪 Tests: No tests were generated for this task.
- 📁 Files: Updated specs/001-book-content-update/tasks.md.
- 🔁 Next prompts: The next step would be to execute the tasks using `/sp.implement`.
- 🧠 Reflection: The task update was successful, adhering to the specified format and organization rules, and incorporating the user's latest input.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script failed, requiring manual PHR creation.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Investigate why `create-phr.sh` failed in this environment and ensure it can be run successfully.
