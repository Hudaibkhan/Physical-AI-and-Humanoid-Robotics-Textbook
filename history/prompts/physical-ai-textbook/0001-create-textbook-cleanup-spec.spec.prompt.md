---
id: 0001
title: Create textbook cleanup spec
stage: spec
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: textbook-cleanup-update
branch: 002-textbook-cleanup-update
user: Muhammad Hudaib Arif
command: /sp.specify
labels: [specification, textbook, cleanup, update]
links:
  spec: specs/002-textbook-cleanup-update/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-textbook-cleanup-update/spec.md
 - specs/002-textbook-cleanup-update/checklists/requirements.md
tests:
 - specs/002-textbook-cleanup-update/spec.md validates against template structure
 - checklists/requirements.md created for quality validation
---

## Prompt

— Updated Specification (Markdown)

Note: update specification in spec.md file inside /specs/001-physical-ai-book**

# **Specification: Physical AI Textbook — Directory Cleanup & Content Update**

## 🎯 Goal
Update the existing Physical AI & Humanoid Robotics Textbook project **without creating new folders** (except where allowed), by cleaning the directory structure, deleting outdated files, renaming folders, updating content, adding missing content, improving module pages, updating sidebar structure, and adding diagrams/images where needed.

---

## ✅ **Scope of Work**

### **1. Directory Cleanup**
Perform the following deletions:

- `docs/final-exam.md` → **delete**
- `docs/glossary.md` → **delete**
- `docs/hardware.md` → **delete**
- `docs/overview.md` → **delete**

Delete entire old module folders:
- `docs/module1-ros2-nervous-system`
- `docs/module2-digital-twin-simulation`
- `docs/module3-ai-brain-nvidia-isaac`
- `docs/module4-vision-language-action-robotics`
- `specs/001-book-content-update`

_No new files created here — just delete._

---

### **2. Folder Rename (No new folder creation)**
Rename existing folders only:

- `docs/ai-brain-isaac` → `docs/module3-ai-brain-isaac`
- `docs/digital-twin-simulation` → `docs/module2-digital-twin-simulation`
- `docs/ros2-nervous-system` → `docs/module1-ros2-nervous-system`
- `docs/vla-robotics` → `docs/module4-vla-robotics`

---

### **3. Content Update (Add/Refine/Review)**

#### **Update intro.md (book front page inside docs/)**
- Add book title
- Add short description
- Add table of contents
- Add module cards preview
- Add hero banner (text + image placeholder)

#### **Update weekly-roadmap.md**
- Add complete 1–13 week roadmap
- Add diagrams if needed
- Add progress tracking structure

#### **Update Additional Materials folder**
For:
- `index.md`
- `cloud.md`
- `hardware.md`
- `final_materials.md`

➡ Add missing content, explanations, examples, diagrams.

#### **Update module index pages**
Add high-quality content to:

- `module1-ros2-nervous-system/index.md`
- `module2-digital-twin-simulation/index.md`
- `module3-ai-brain-isaac/index.md`
- `module4-vla-robotics/index.md`

Content includes:
- Module intro
- Learning objectives
- Diagram placeholders
- Summary of chapters
- Links to chapters

#### **Review all module chapters**
Check chapters (chapter1.md, chapter2.md, etc.) for:
- Missing content
- Broken formatting
- Missing diagrams
- Clarification needed
- Ensure consistent theme
- Fix headings and titles

---

### **4. Sidebar Structure Update**

Replace current chaotic sidebar with this new structure:

Physical AI & Humanoid Robotics Textbook
│
├── Module 1: ROS 2 Nervous System
├── Module 2: Digital Twin Simulation
├── Module 3: AI Brain (NVIDIA Isaac)
├── Module 4: Vision-Language-Action Robotics
│
├── Capstone Project
├── Additional Materials
└── Weekly Roadmap



Update `sidebars.js` accordingly after folder renaming and deletions.

---

### **5. UI Updates (Frontend)**

#### **Create `src/pages/index.tsx` (allowed)**
This is the book's front page and must include:

- Book title
- Short description
- "Start Reading" button
- Module cards (4 modules)
  - icon
  - title
  - short intro
  - direct link to module
- Clean UI matching textbook color theme

---

### **6. Theme & Branding**
Apply a consistent color theme across:

- All docs markdown pages
- Module headers
- Code highlight style
- Front page
- Custom CSS (`src/css/custom.css`)

Theme style:
- Futuristic robotics
- Blue/Purple Neon Accent
- Modern education layout

---

### **7. Update /specs/001-physical-ai-book**
Do NOT create new files. Only **update** these:

- `plan.md`
- `quickstart.md`
- `research.md`
- `spec.md`
- `tasks.md`

Update them to reflect:
- New folder structure
- Updated content plan
- Updated roadmap
- Updated sidebar
- Removal of deleted items

---

## ❌ Out of Scope
- Do NOT cre

## Response snapshot

# Feature Specification: Physical AI Textbook — Directory Cleanup & Content Update

**Feature Branch**: `002-textbook-cleanup-update`
**Created**: 2025-12-07
**Status**: Draft
**Input**: User description: "— Updated Specification (Markdown)

Note: update specification in spec.md file inside /specs/001-physical-ai-book**

# **Specification: Physical AI Textbook — Directory Cleanup & Content Update**

## 🎯 Goal
Update the existing Physical AI & Humanoid Robotics Textbook project **without creating new folders** (except where allowed), by cleaning the directory structure, deleting outdated files, renaming folders, updating content, adding missing content, improving module pages, updating sidebar structure, and adding diagrams/images where needed.

---

## ✅ **Scope of Work**

### **1. Directory Cleanup**
Perform the following deletions:

- `docs/final-exam.md` → **delete**
- `docs/glossary.md` → **delete**
- `docs/hardware.md` → **delete**
- `docs/overview.md` → **delete**

Delete entire old module folders:
- `docs/module1-ros2-nervous-system`
- `docs/module2-digital-twin-simulation`
- `docs/module3-ai-brain-nvidia-isaac`
- `docs/module4-vision-language-action-robotics`
- `specs/001-book-content-update`

_No new files created here — just delete._

---

### **2. Folder Rename (No new folder creation)**
Rename existing folders only:

- `docs/ai-brain-isaac` → `docs/module3-ai-brain-isaac`
- `docs/digital-twin-simulation` → `docs/module2-digital-twin-simulation`
- `docs/ros2-nervous-system` → `docs/module1-ros2-nervous-system`
- `docs/vla-robotics` → `docs/module4-vla-robotics`

---

### **3. Content Update (Add/Refine/Review)**

#### **Update intro.md (book front page inside docs/)**
- Add book title
- Add short description
- Add table of contents
- Add module cards preview
- Add hero banner (text + image placeholder)

#### **Update weekly-roadmap.md**
- Add complete 1–13 week roadmap
- Add diagrams if needed
- Add progress tracking structure

#### **Update Additional Materials folder**
For:
- `index.md`
- `cloud.md`
- `hardware.md`
- `final_materials.md`

➡ Add missing content, explanations, examples, diagrams.

#### **Update module index pages**
Add high-quality content to:

- `module1-ros2-nervous-system/index.md`
- `module2-digital-twin-simulation/index.md`
- `module3-ai-brain-isaac/index.md`
- `module4-vla-robotics/index.md`

Content includes:
- Module intro
- Learning objectives
- Diagram placeholders
- Summary of chapters
- Links to chapters

#### **Review all module chapters**
Check chapters (chapter1.md, chapter2.md, etc.) for:
- Missing content
- Broken formatting
- Missing diagrams
- Clarification needed
- Ensure consistent theme
- Fix headings and titles

---

### **4. Sidebar Structure Update**

Replace current chaotic sidebar with this new structure:

Physical AI & Humanoid Robotics Textbook
│
├── Module 1: ROS 2 Nervous System
├── Module 2: Digital Twin Simulation
├── Module 3: AI Brain (NVIDIA Isaac)
├── Module 4: Vision-Language-Action Robotics
│
├── Capstone Project
├── Additional Materials
└── Weekly Roadmap



Update `sidebars.js` accordingly after folder renaming and deletions.

---

### **5. UI Updates (Frontend)**

#### **Create `src/pages/index.tsx` (allowed)**
This is the book's front page and must include:

- Book title
- Short description
- "Start Reading" button
- Module cards (4 modules)
  - icon
  - title
  - short intro
  - direct link to module
- Clean UI matching textbook color theme

---

### **6. Theme & Branding**
Apply a consistent color theme across:

- All docs markdown pages
- Module headers
- Code highlight style
- Front page
- Custom CSS (`src/css/custom.css`)

Theme style:
- Futuristic robotics
- Blue/Purple Neon Accent
- Modern education layout

---

### **7. Update /specs/001-physical-ai-book**
Do NOT create new files. Only **update** these:

- `plan.md`
- `quickstart.md`
- `research.md`
- `spec.md`
- `tasks.md`

Update them to reflect:
- New folder structure
- Updated content plan
- Updated roadmap
- Updated sidebar
- Removal of deleted items

---

## ❌ Out of Scope
- Do NOT cre"

## Project Context

You are updating the existing Physical AI & Humanoid Robotics Textbook project by cleaning the directory structure, deleting outdated files, renaming folders, updating content, adding missing content, improving module pages, updating sidebar structure, and adding diagrams/images where needed. The goal is to create a cleaner, more organized, and more comprehensive textbook with improved user experience.

## Target Audience

- Q4 Generative AI diploma students
- Beginner-to-intermediate AI developers
- Learners comfortable with AI models but new to robots
- Teachers building robotics courses
- Self-learners who want to enter robotics with Windows/PowerShell workflows

## Primary Focus

- Clean up the directory structure and remove outdated files
- Rename folders to follow a consistent naming convention
- Update content to improve quality and completeness
- Create a better organized sidebar structure
- Implement a consistent futuristic robotics theme with blue/purple neon accents
- Create a proper front page for the textbook

## Out of Scope

- Creating new functional features beyond the specified cleanup and updates
- Adding new technical content that wasn't already planned
- Changing the core educational content of the modules
- Modifying the underlying Docusaurus framework configuration

## User Scenarios & Testing

### User Story 1 - Navigating the Textbook (Priority: P1)

Students will be able to navigate the textbook using a clean, organized sidebar that follows a logical module structure, making it easier to find specific content and follow the learning path.

**Why this priority**: Navigation is the primary way students interact with the textbook, and a chaotic or inconsistent structure significantly impacts the learning experience.

**Independent Test**: Can be fully tested by verifying that the sidebar structure is clear, organized, and follows the specified hierarchy: Module 1-4, Capstone Project, Additional Materials, Weekly Roadmap.

**Acceptance Scenarios**:

1. **Given** a student opens the textbook, **When** they look at the sidebar, **Then** they see a clear, organized structure with the specified modules in order.
2. **Given** a student wants to access Module 3 content, **When** they click on "Module 3: AI Brain (NVIDIA Isaac)" in the sidebar, **Then** they are taken to the correct module page.
3. **Given** a student wants to see additional materials, **When** they click on "Additional Materials" in the sidebar, **Then** they see the additional resources section.

---

### User Story 2 - Accessing Front Page Content (Priority: P1)

Students will be able to access a well-designed front page that provides an overview of the textbook, module cards, and clear navigation to begin learning.

**Why this priority**: The front page is the entry point to the textbook and should provide an engaging introduction to the content and clear pathways to begin learning.

**Independent Test**: Can be fully tested by creating the `src/pages/index.tsx` file with the specified elements and verifying it displays correctly with the proper theme.

**Acceptance Scenarios**:

1. **Given** a student visits the textbook website, **When** they land on the front page, **Then** they see the book title, description, and module cards.
2. **Given** a student sees the module cards on the front page, **When** they click on a module card, **Then** they are taken to the corresponding module page.
3. **Given** a student sees the "Start Reading" button on the front page, **When** they click it, **Then** they are taken to the beginning of the textbook content.

---

### User Story 3 - Accessing Updated Content (Priority: P2)

Students will be able to access updated content in the textbook that includes complete information, proper diagrams, and consistent formatting throughout all modules.

**Why this priority**: Complete and well-formatted content is essential for effective learning and comprehension of complex robotics concepts.

**Independent Test**: Can be fully tested by reviewing all module index pages and chapters to ensure they contain complete content with proper headings, formatting, and diagram placeholders.

**Acceptance Scenarios**:

1. **Given** a student navigates to a module index page, **When** they read the content, **Then** they find complete information including module intro, learning objectives, and summary of chapters.
2. **Given** a student reads a chapter within a module, **When** they look for diagrams or examples, **Then** they find appropriate placeholders or actual diagrams where needed.
3. **Given** a student moves between different chapters, **When** they notice the formatting, **Then** they see consistent styling and structure throughout.

---

### Edge Cases

- What happens when a student tries to access a page that was deleted during the cleanup? The system should handle this gracefully with a proper 404 page or redirect.
- How does the updated sidebar handle different screen sizes and mobile devices? The sidebar should be responsive and accessible on all devices.
- What if a student bookmarks a page that gets renamed during the update? Clear redirects should be implemented for critical pages.
- How are students affected if the new theme conflicts with their browser settings or accessibility needs? The theme should maintain accessibility standards.

## Requirements

### Functional Requirements

- **FR-001**: The system MUST delete the specified outdated files: `docs/final-exam.md`, `docs/glossary.md`, `docs/hardware.md`, `docs/overview.md`.
- **FR-002**: The system MUST delete the entire old module folders: `docs/module1-ros2-nervous-system`, `docs/module2-digital-twin-simulation`, `docs/module3-ai-brain-nvidia-isaac`, `docs/module4-vision-language-action-robotics`, `specs/001-book-content-update`.
- **FR-003**: The system MUST rename the specified folders: `docs/ai-brain-isaac` → `docs/module3-ai-brain-isaac`, `docs/digital-twin-simulation` → `docs/module2-digital-twin-simulation`, `docs/ros2-nervous-system` → `docs/module1-ros2-nervous-system`, `docs/vla-robotics` → `docs/module4-vla-robotics`.
- **FR-004**: The system MUST update `docs/intro.md` with book title, short description, table of contents, module cards preview, and hero banner.
- **FR-005**: The system MUST update `docs/weekly-roadmap.md` with a complete 1-13 week roadmap and diagrams.
- **FR-006**: The system MUST update the Additional Materials content including `index.md`, `cloud.md`, `hardware.md`, `final_materials.md` with missing content, explanations, examples, and diagrams.
- **FR-007**: The system MUST update all module index pages with high-quality content including module intro, learning objectives, diagram placeholders, summary of chapters, and links to chapters.
- **FR-008**: The system MUST review all module chapters for missing content, broken formatting, missing diagrams, and clarification needs.
- **FR-009**: The system MUST update `sidebars.js` to reflect the new organized structure.
- **FR-010**: The system MUST create `src/pages/index.tsx` with the specified elements: book title, description, "Start Reading" button, module cards with icons, titles, intros, and direct links.
- **FR-011**: The system MUST apply a consistent futuristic robotics theme with blue/purple neon accents across all docs markdown pages, module headers, code highlight style, front page, and custom CSS.
- **FR-012**: The system MUST update the files in `/specs/001-physical-ai-book` to reflect the new structure: `plan.md`, `quickstart.md`, `research.md`, `spec.md`, `tasks.md`.
- **FR-013**: The system MUST ensure all content follows a consistent theme and formatting style.
- **FR-014**: The system MUST ensure all links and navigation elements work correctly after the reorganization.
- **FR-015**: The system MUST ensure no data loss occurs during the deletion and renaming operations.

### Key Entities

- **Outdated File**: A file that is no longer needed in the textbook structure (e.g., `docs/final-exam.md`).
- **Module Folder**: A directory containing content for a specific module of the textbook.
- **Module Index Page**: The main page for each module containing intro, objectives, and chapter summaries.
- **Sidebar Structure**: The navigation menu on the textbook website.
- **Front Page**: The main landing page of the textbook (`src/pages/index.tsx`).
- **Theme**: The visual styling applied across the textbook (futuristic robotics with blue/purple neon accents).
- **Specification File**: Documentation files in `/specs/001-physical-ai-book` that need updating.

## Success Criteria

### Measurable Outcomes

- **SC-001**: All specified outdated files MUST be successfully deleted without errors.
- **SC-002**: All specified module folders MUST be successfully renamed without errors.
- **SC-003**: The `docs/intro.md` file MUST contain all specified elements: book title, description, table of contents, module cards preview, and hero banner.
- **SC-004**: The `docs/weekly-roadmap.md` file MUST contain a complete 1-13 week roadmap with diagrams.
- **SC-005**: All Additional Materials files MUST contain updated content with explanations, examples, and diagrams.
- **SC-006**: All module index pages MUST contain complete content with module intro, learning objectives, diagram placeholders, chapter summaries, and links.
- **SC-007**: The `sidebars.js` file MUST reflect the new organized structure.
- **SC-008**: The `src/pages/index.tsx` file MUST be created with all specified elements.
- **SC-009**: The consistent futuristic robotics theme with blue/purple neon accents MUST be applied across all pages.
- **SC-010**: All specification files in `/specs/001-physical-ai-book` MUST be updated to reflect the new structure.
- **SC-011**: All navigation links MUST work correctly after the reorganization.
- **SC-012**: The Docusaurus build process MUST complete with zero errors after all changes.
- **SC-013**: The textbook navigation MUST follow the specified sidebar structure: Module 1-4, Capstone Project, Additional Materials, Weekly Roadmap.
- **SC-014**: All module chapters MUST be reviewed and updated for missing content, formatting, and diagrams.
- **SC-015**: The textbook MUST maintain all original educational content while improving organization and presentation.

## Assumptions

- The user has proper access rights to delete and rename files in the repository
- The Docusaurus framework is properly configured and will continue to work after the reorganization
- The content in the old module folders can be safely deleted as it's outdated
- The content in the new module folders (ai-brain-isaac, etc.) can be moved to the renamed folders
- The existing content in the textbook can be enhanced without changing its core educational value
- The theme changes will not break existing functionality

## Dependencies

- **Docusaurus**: The documentation framework that will serve the textbook
- **Git**: Version control system for tracking changes during the reorganization
- **Node.js/npm**: Required for building the Docusaurus site after changes
- **File system access**: Required to delete, rename, and create files

## Definition of Done

- [ ] All specified outdated files have been deleted
- [ ] All specified folders have been renamed
- [ ] The `docs/intro.md` file has been updated with all required elements
- [ ] The `docs/weekly-roadmap.md` file has been updated with a complete roadmap
- [ ] All Additional Materials files have been updated with content
- [ ] All module index pages have been updated with complete content
- [ ] All module chapters have been reviewed and updated
- [ ] The `sidebars.js` file has been updated with the new structure
- [ ] The `src/pages/index.tsx` file has been created with all required elements
- [ ] The futuristic robotics theme has been applied consistently
- [ ] All specification files in `/specs/001-physical-ai-book` have been updated
- [ ] The Docusaurus site builds without errors
- [ ] All navigation links work correctly
- [ ] The textbook follows the specified sidebar structure

## Outcome

- ✅ Impact: Created comprehensive specification for textbook cleanup and content update project
- 🧪 Tests: Specification validates against template structure
- 📁 Files: specs/002-textbook-cleanup-update/spec.md, specs/002-textbook-cleanup-update/checklists/requirements.md
- 🔁 Next prompts: /sp.plan to create implementation plan
- 🧠 Reflection: The specification covers all required aspects of the textbook cleanup project with clear user stories, requirements, and success criteria

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation planning with /sp.plan