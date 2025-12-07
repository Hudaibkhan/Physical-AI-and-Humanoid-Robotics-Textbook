---
ID: 002
TITLE: Create Physical AI Book Plan
STAGE: plan
DATE_ISO: 2025-12-05
SURFACE: agent
MODEL: gemini-2.5-flash
FEATURE: physical-ai-book
BRANCH: 001-physical-ai-book
USER:
COMMAND: /sp.plan
LABELS: ["planning", "textbook"]
LINKS:
  SPEC: "specs/001-physical-ai-book/spec.md"
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - specs/001-physical-ai-book/plan.md
  - specs/001-physical-ai-book/research.md
  - specs/001-physical-ai-book/quickstart.md
TESTS_YAML:
  - Plan created successfully
PROMPT_TEXT: |
  — Implementation Plan (Physical AI & Humanoid Robotics Textbook)
  ✅ Goal

  Turn the approved Specification into a full textbook by defining:

  Work breakdown

  Chapter creation order

  Folder structure

  Execution phases

  Dependencies

  Expected outputs for each cycle

  1. Project Structure Setup (Week 0)
  1.1 Create Docusaurus Folder Structure
  /docs/
    /module1/
    /module2/
    /module3/
    /module4/
    /capstone/
    /appendix/
    overview.md
    weekly-roadmap.md
    hardware.md
    glossary.md
    final-exam.md

  1.2 Create Base Templates

  Each chapter uses the following template:

  # Title
  ## Learning Goals
  ## Prerequisites
  ## Key Concepts
  ## DiagrOS)

  Photorealistic digital twins

  Synthetic data

  Perception stack

  VSLAM, SLAM, AprilTags

  Nav2 for humanoids

  Module 4 (VLA)

  LLM-driven robotics

  Voice → Whisper → LLM → ROS 2

  Planning + perception + action

  High-level control systems

  Output:

  Chapters 9–13 fully written

  🟪 Phase 4 — ams
  ## Examples
  ## Hands-on Exercises
  ## Assignments
  ## Summary

  2. Production Phases (4 Phases)
  🟦 Phase 1 — Skeleton Draft (All Chapters Outline)

  Goal: Create outline-only versions of all 17 chapters.

  Tasks:

  For each chapter:

  Write headings

  Add placeholder section names (no content)

  Add callouts for diagrams

  Create subsections for exercises & examples

  Generate hero page + short course overview

  Build weekly roadmap layout (empty)

  Output:

  All 17 chapters created (outline version)

  Docusaurus builds successfully

  🟩 Phase 2 — Core Content Writing (Modules 1 & 2)

  Focus: ROS 2 + Gazebo + Unity

  Module 1 (ROS 2)

  ROS 2 concepts

  Nodes/Topics/Services/Actions

  Humanoid URDF

  Sensor plugins

  PowerShell-based workflows

  Module 2 (Simulation)

  Gazebo physics

  Sensors

  World-building

  Unity visualization overview

  Output:

  Chapters 1–8 (full content)

  60–80 pages ready

  🟥 Phase 3 — Advanced Robotics (Modules 3 & 4)

  Focus: Isaac Sim + VLA systems + Navigation

  Module 3 (Isaac Sim & Isaac RFinalization (Capstone + Exams + Glossary)

  Focus: Wrap-up + student assessment tools

  Tasks:

  Write Capstone Project ("Autonomous Humanoid Robot")

  Add Hardware architecture & lab environments (Windows + Cloud)

  Add Cloud-native robotics workflows

  Create Glossary (120–160 terms)

  Add Final Exam (40–60 questions)

  Add Grading Rubric

  Polish diagrams & visuals

  Final proofread & readability check

  Output:

  Chapters 14–17

  Full textbook ready (200–250 pages once expanded)

  3. Chapter-by-Chapter Plan (Detailed)
  Module 1 — ROS 2 Nervous System

  Introduction to Physical AI

  Digital → Embodied Intelligence

  ROS 2 System Overview

  ROS Nodes/Topics/Services/Actions

  URDF & Humanoid Structure

  Module 2 — Digital Twin

  Gazebo Simulation Fundamentals

  Physics & Sensors

  Unity for Robot Visualization

  Module 3 — AI-Robot Brain

  Isaac Sim Fundamentals

  Isaac ROS Perception

  Navigation (Nav2 + Biped basics)

  Module 4 — VLA Systems

  VLA Architecture

  Voice-to-Action Pipeline

  Final Materials

  Capstone Project

  Hardware & Lab Setup

  Cloud Robotics Environments

  Glossary, Appendix, References, Final Exam

  4. Collaboration & Execution Workflow (Spec-Kit)
  Cycle Process
  Cycle    Command    Purpose
  1    /sp.specify    Write specification (done ✔)
  2    /sp.plan    Create this implementation plan
  3    /sp.tasks    Convert plan → atomic tasks
  4    /sp.clarify    Identify gaps before writing
  5+    /sp.implement    Generate chapters one-by-one
  5. Technical Constraints in Plan

  All terminal commands use PowerShell

  No Ubuntu/Linux examples

  Must support:

  ROS 2 Humble (Windows MSI)

  Gazebo Fortress/Garden

  Unity HDRP

  Isaac Sim (Windows or Cloud)

  No heavy hardware required until Module 3

  6. Timeline Breakdown
  Total Time: 3–7 cycles
  Phase    Deliverable    ETA
  Phase 1    Chapter skeletons    2–3 days
  Phase 2    Modules 1 & 2    5–7 days
  Phase 3    Modules 3 & 4    5–8 days
  Phase 4    Capstone + Extras    3–5 days
  7. Definition of Done

  ✓ 17 chapters complete
  ✓ Glossary, exam, rubric added
  ✓ Hero page + overview + roadmap
  ✓ Docusaurus builds successfully
  ✓ ROS + Gazebo + Isaac examples validated
  ✓ All content readable (grade ≤ 8 English)
  ✓ No placeholders, no missing sections

  Cleanup Request (English Version)

  Please delete the previously created folder 001-ros2-module1 because it already contains old specification, tasks, and planning work.
  We will create a new updated version based on the latest finalized specification.
RESPONSE_TEXT: |
  Manually created the `specs/001-physical-ai-book/plan.md`, `specs/001-physical-ai-book/research.md`, and `specs/001-physical-ai-book/quickstart.md` files, incorporating the provided implementation plan.
OUTCOME: Implementation plan created.
EVALUATION: The plan is ready for the task generation phase.
---
