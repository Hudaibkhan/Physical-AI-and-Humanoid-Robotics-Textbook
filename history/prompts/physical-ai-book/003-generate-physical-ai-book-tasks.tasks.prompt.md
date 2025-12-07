---
ID: 003
TITLE: Generate Physical AI Book Tasks
STAGE: tasks
DATE_ISO: 2025-12-05
SURFACE: agent
MODEL: gemini-2.5-flash
FEATURE: physical-ai-book
BRANCH: 001-physical-ai-book
USER:
COMMAND: /sp.tasks
LABELS: ["tasks", "textbook"]
LINKS:
  SPEC: "specs/001-physical-ai-book/spec.md"
  PLAN: "specs/001-physical-ai-book/plan.md"
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - specs/001-physical-ai-book/tasks.md
TESTS_YAML:
  - Tasks generated successfully
  - All tasks follow checklist format
  - Tasks organized by user story and phases
PROMPT_TEXT: |
  — Atomic Task Breakdown (Physical AI & Humanoid Robotics Textbook)
  ✔ Auto-Includes: Specification Review + Plan Review + Cleanup Instructions

  This task list is designed for Spec-Kit automatic execution, and it includes:

  Review of previous Specification

  Review of Implementation Plan

  Cleanup of old folder (001-ros2-module1)

  Creation of new Overview Page (Premium modern design)

  Creation of all textbook chapters

  Premium book design pass

  Docusaurus structure tasks

  Task order optimized for multi-cycle execution

  ✅ Before Tasks Start — Automatic Review & Cleanup (Meta-Tasks)
  acing & visual hierarchy

  🟦 Module B — Overview Page (Modern Premium UI)
  Task B1 — Create Modern Overview Page
  Create overview.md displaying Module Cards:
  - Module 1: ROS 2 Nervous System
  - Module 2: Digital Twin Simulation
  - Module 3: NVIDIA Isaac AI Brain
  - Module 4: Vision-Language-Action Robotics
  Each card contains:
     • short description
     • icon
     • quick start link
     • estimated difficulty

  Layout:
  - Grid of large cards
  - Modern rounded design
  - Soft shadow
  - Brand colors
  - Minimalistic clean UI

  Task B2 — Make Module Cards Clickable
  Each card links to the first chapter of that Task M0 — Review Previous Work
  Review the final approved Specification and Implementation Plan.
  Confirm all chapters, modules, constraints, and outputs are aligned.

  Task M1 — Cleanup Old Module Work
  Delete the previous folder: 001-ros2-module1
  (remove old spec, plan, tasks, and chapter attempts)

  �� Module A — Project Structure & Branding Setup
  Task A1 — Create Docusaurus Folder Structure
  Create the following folders:
   /docs/module1/
   /docs/module2/
   /docs/module3/
   /docs/module4/
   /docs/capstone/
   /docs/appendix/

  Create pages:
   overview.md
   weekly-roadmap.md
   hardware.md
   glossary.md
   final-exam.md

  Task A2 — Create Chapter Template
  Add a reusable markdown template with:
  # Title
  ## Learning Goals
  ## Prerequisites
  ## Key Concepts
  ## Diagrams
  ## Examples
  ## Hands-on Exercises
  ## Assignments
  ## Summary

  Task A3 — Design Premium “Book Look”
  Add consistent premium design:
  - Hero banners
  - Section dividers
  - Color tokens
  - Icons per module
  - Typography scale
  - Modern minimal layout
  - Clean spmodule.

  Task B3 — Add “Full Book Premium Look”
  Add global styles:
  - consistent margins
  - premium separators
  - callout blocks
  - info/warning boxes
  - quote styles
  - diagram placeholders

  🟦 Phase 1 Tasks — Skeleton Draft of All Chapters
  Task C1 — Generate Outline for All 17 Chapters
  For each chapter:
  - Create file
  - Add title
  - Add empty sections (from template)
  - Add diagram callouts
  - Add exercise placeholders
  - No content yet

  Task C2 — Validate Docusaurus Build
  Ensure all outline pages build with zero errors.

  🟩 Phase 2 Tasks — Full Content for Modules 1 & 2
  Task D1 — Write Module 1 (ROS 2) Full Content

  Includes:

  - Nodes, Topics, Services, Actions
  - Humanoid URDF
  - Sensors
  - PowerShell workflows
  - Launch files

  Task D2 — Write Module 2 (Simulation) Full Content

  Includes:

  - Gazebo physics
  - Sensors
  - World building
  - Unity visualization

  Task D3 — Add Diagrams for Modules 1–2
  Add placeholders and descriptions of diagrams.

  🟥 Phase 3 Tasks — Full Content for Modules 3 & 4
  Task E1 — Write Module 3 (Isaac Sim & Isaac ROS)

  Includes:

  - Digital twins
  - Synthetic data
  - Perception stack
  - VSLAM, SLAM, AprilTags
  - Nav2 path planning (humanoid context)

  Task E2 — Write Module 4 (VLA Robotics)

  Includes:

  - VLA architecture
  - Whisper → LLM → ROS 2 actions
  - Natural language planning
  - Task decomposition

  🟪 Phase 4 Tasks — Capstone, Glossary, Exam, Final
  Task F1 — Write Capstone Project
  Autonomous Humanoid Robot project
  with objectives, requirements, milestones

  Task F2 — Hardware & Cloud Lab Setup
  Full hardware tiers (low-end, workstation, cloud)

  Task F3 — Glossary
  Create 120–160 robotics terms with simple definitions

  Task F4 — Final Exam + Rubric
  40–60 questions
  Grading system
  Performance rubric

  Task F5 — Final Polishing Pass
  Proofread
  Check readability (grade ≤ 8)
  Ensure all diagrams referenced
  Ensure all pages build

  🎯 FINAL TASK (Auto-Finish)
  Task Z — Certification of Completion
  Confirm:
  ✓ 17 chapters complete
  ✓ Overview page modern + clickable
  ✓ Premium book layout applied
  ✓ Capstone, exam, glossary completed
  ✓ All modules functional
  ✓ Docusaurus build OK
RESPONSE_TEXT: |
  Generated the `specs/001-physical-ai-book/tasks.md` file, detailing atomic tasks for the textbook's implementation, organized by phases and user stories, and including cleanup instructions for the old module work. The task list is ready for execution.
OUTCOME: Atomic tasks generated and ready for implementation.
EVALUATION: The task list is complete and follows the specified format.
---
