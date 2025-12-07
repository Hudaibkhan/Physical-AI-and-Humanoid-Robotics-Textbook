---
id: 0002
title: Create implementation plan
stage: plan
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: textbook-cleanup-update
branch: 002-textbook-cleanup-update
user: Muhammad Hudaib Arif
command: /sp.plan
labels: [planning, implementation, textbook, cleanup]
links:
  spec: specs/002-textbook-cleanup-update/spec.md
  plan: specs/002-textbook-cleanup-update/plan.md
  research: specs/002-textbook-cleanup-update/research.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-textbook-cleanup-update/plan.md
 - specs/002-textbook-cleanup-update/research.md
 - specs/002-textbook-cleanup-update/data-model.md
 - specs/002-textbook-cleanup-update/quickstart.md
 - specs/002-textbook-cleanup-update/contracts/textbook-api.yaml
 - .specify/memory/constitution.md
tests:
 - Implementation plan validates against constitution requirements
 - Research document addresses all unknowns from spec
 - Data model captures content structure accurately
 - Quickstart guide provides clear implementation steps
 - API contracts defined for textbook navigation
---

## Prompt

Goal:
Update the existing Physical AI & Humanoid Robotics textbook repo by modifying current files, adding missing content, deleting unused folders, renaming directories, updating sidebar structure, and improving the book front page. No new duplicate folders/files should be created—everything must update in-place.

Plan:

1. **Clean Up & Rename Structure**
   - Delete marked files/folders.
   - Rename directories to new module naming (module1–module4).
   - Ensure all referenced paths update accordingly.
   - Update `sidebars.js` to reflect the new clean structure.

2. **Content Review & Completion**
   - Review all existing docs inside `/docs`.
   - Add missing content to:
     - `weekly-roadmap.md`
     - `additional-materials/*`
     - Module index files (module1–module4)
     - Capstone project chapter
   - Add diagrams/images where appropriate.
   - Ensure consistent formatting, headings, and explanations.

3. **Intro & Book Front Page**
   - Build a strong `intro.md` with title, description, goals, and overview.
   - Create `/src/pages/index.tsx`:
     - Book title, subtitle, short description.
     - "Start Reading" button.
     - Module cards (4 modules) with icon, name, description & direct links.
     - Follow single-color theme.

4. **Sidebar Reconstruction**
   - Replace old sidebar with new structure:
     - Physical AI & Humanoid Robotics Textbook
       - Module 1
       - Module 2
       - Module 3
       - Module 4
       - Capstone Project
       - Additional Materials
       - Weekly Roadmap
   - Remove deleted folders from sidebar.

5. **Specs Folder Updates**
   - Update existing `spec.md`, `plan.md`, `quickstart.md`, `tasks.md`, `research.md` inside `001-physical-ai-book`.
   - Remove obsolete `001-book-content-update` folder.

6. **Theme & Visual Consistency**
   - Apply a unified color theme across pages.
   - Ensure all diagrams/images follow clean design.
   - Update CSS in `custom.css` as needed.

7. **Final

## Response snapshot

# Implementation Plan: Physical AI Textbook — Directory Cleanup & Content Update

**Branch**: `002-textbook-cleanup-update` | **Date**: 2025-12-07 | **Spec**: [specs/002-textbook-cleanup-update/spec.md](specs/002-textbook-cleanup-update/spec.md)
**Input**: Feature specification from `/specs/002-textbook-cleanup-update/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan addresses the comprehensive cleanup and update of the Physical AI & Humanoid Robotics textbook repository. The implementation involves removing outdated files and folders, renaming existing module directories to follow a consistent naming convention, updating content across multiple files, creating a new front page, restructuring the sidebar navigation, and applying a consistent futuristic robotics theme. The project maintains the educational content while improving organization, navigation, and visual consistency.

## Technical Context

**Language/Version**: Markdown, TypeScript/JSX (for Docusaurus)
**Primary Dependencies**: Docusaurus v3, Node.js, npm
**Storage**: File-based documentation system
**Testing**: Manual verification of Docusaurus build and navigation
**Target Platform**: GitHub Pages deployment via GitHub Actions
**Project Type**: Static documentation website (web)
**Performance Goals**: Fast loading pages, responsive navigation, accessible content
**Constraints**: Must maintain Docusaurus compatibility, preserve existing educational content, ensure all links work after reorganization
**Scale/Scope**: 4 textbook modules, 10+ content files, 1 front page, updated sidebar

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Spec-Driven Writing**: ✅ All changes initiated through specifications via Spec-Kit Plus
- **Technical Reliability**: ✅ All documentation and code examples must be technically accurate and verifiable
- **Beginner-Friendly + Professional Tone**: ✅ Content must remain understandable for beginners while maintaining professional quality
- **Documentation Quality**: ✅ Writing style follows high-quality open-source documentation standards (Docusaurus, Next.js, Python docs)
- **Consistency**: ✅ Terminology, formatting, and structure must remain consistent across all modules and chapters
- **Book Requirements**: ✅ Maintains minimum 4 modules with proper structure (Module → Chapters → Sections)
- **Technical Constraints**: ✅ Output only Markdown compatible with Docusaurus, follows Spec-Kit workflow

## Project Structure

### Documentation (this feature)

```text
specs/002-textbook-cleanup-update/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docs/
├── module1-ros2-nervous-system/      # Renamed from ros2-nervous-system
├── module2-digital-twin-simulation/  # Renamed from digital-twin-simulation
├── module3-ai-brain-isaac/           # Renamed from ai-brain-isaac
├── module4-vla-robotics/             # Renamed from vla-robotics
├── additional-materials/             # Updated content
│   ├── index.md
│   ├── cloud.md
│   ├── hardware.md
│   └── final_materials.md
├── weekly-roadmap.md                 # Updated with 1-13 week roadmap
├── intro.md                          # Updated with title, description, etc.
└── capstone.md                       # Updated content

src/
├── pages/
│   └── index.tsx                     # New front page with module cards
├── components/                       # Docusaurus components
└── css/
    └── custom.css                    # Updated theme with futuristic robotics style

sidebars.js                            # Updated navigation structure

specs/
├── 001-physical-ai-book/             # Updated spec files
│   ├── spec.md
│   ├── plan.md
│   ├── research.md
│   ├── quickstart.md
│   └── tasks.md
└── 002-textbook-cleanup-update/      # This feature's specs
    └── [files listed above]
```

**Structure Decision**: The project follows the Docusaurus documentation structure with updated module naming conventions and enhanced front page. The structure maintains the existing educational content while improving organization and navigation.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple file updates | Required for comprehensive textbook update | Updating single files would not achieve the overall goal of organization and consistency |
| Directory renaming | Needed for consistent module naming | Keeping old names would maintain the chaotic structure that was identified as problematic |

## Outcome

- ✅ Impact: Created comprehensive implementation plan for textbook cleanup and update project
- 🧪 Tests: Plan validates against constitution requirements and feature specification
- 📁 Files: specs/002-textbook-cleanup-update/plan.md, research.md, data-model.md, quickstart.md, contracts/textbook-api.yaml
- 🔁 Next prompts: /sp.tasks to generate specific implementation tasks
- 🧠 Reflection: The plan addresses all required aspects of the textbook cleanup project with clear technical approach, constitution compliance, and structured implementation approach

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate specific tasks with /sp.tasks