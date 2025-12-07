---
description: "Task list for Physical AI & Humanoid Robotics Textbook Update"
---

# Tasks: Physical AI & Humanoid Robotics Textbook Update

**Input**: Design documents from `/specs/001-book-content-update/`
**Prerequisites**: plan.md (required), spec.md (required)

**Tests**: Tests are NOT explicitly requested.

**Organization**: Tasks are grouped by logical sections to facilitate sequential and parallel implementation.

## Format: `[ID] [P?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `docs/`, `static/` at repository root

## Phase 1: Sidebar Structure Update

- [x] T001 Update `sidebars.js` to replace old sidebar order with new hierarchical structure: Physical AI & Humanoid Robotics Textbook, Module 1-4, Capstone Project, Additional Materials, Weekly Roadmap
- [x] T002 Remove duplicated or outdated sidebar entries from `sidebars.js`
- [x] T003 Ensure consistent ordering and naming across all sidebar configurations in `sidebars.js`

---

## Phase 2: Header & Footer Enhancements

### Header
- [x] T004 Add company/logo image to `static/img/logo.png` (create if doesn't exist)
- [ ] T005 Integrate logo into Docusaurus header configuration in `docusaurus.config.js`
- [x] T006 Ensure header is responsive by adjusting CSS in `src/css/custom.css` or theme overrides

### Footer
- [ ] T007 Implement footer component (e.g., `src/components/Footer.js`) or update Docusaurus theme layout
- [x] T008 Add links to all modules in the footer (referencing `docs/moduleX-name/index.md`)
- [x] T009 Add links to Additional Materials (`docs/additional-materials/index.md`) and Weekly Roadmap (`docs/weekly-roadmap/index.md`) in the footer
- [x] T010 Ensure consistent design and theme for the footer by adjusting styles in `src/css/custom.css`

---

## Phase 3: Review & Improve Existing `/docs/` Content

- [x] T011 Review and improve clarity, flow, and formatting of `docs/Physical-AI-and-Humanoid-Robotics-Textbook/intro.md`
- [x] T-PREREQ-01 Create missing `docs/Physical-AI-and-Humanoid-Robotics-Textbook/intro.md` file
- [ ] T012 Review and improve clarity, flow, and formatting of `docs/module1-ros2-nervous-system/index.md`
- [x] T-PREREQ-02 Create missing `docs/module1-ros2-nervous-system/index.md` file
- [ ] T013 Review and improve clarity, flow, and formatting of `docs/module2-digital-twin-simulation/index.md`
- [x] T-PREREQ-03 Create missing `docs/module2-digital-twin-simulation/index.md` file
- [ ] T014 Review and improve clarity, flow, and formatting of `docs/module3-ai-brain-nvidia-isaac/index.md`
- [x] T-PREREQ-04 Create missing `docs/module3-ai-brain-nvidia-isaac/index.md` file
- [ ] T015 Review and improve clarity, flow, and formatting of `docs/module4-vision-language-action-robotics/index.md`
- [x] T-PREREQ-05 Create missing `docs/module4-vision-language-action-robotics/index.md` file
- [ ] T016 Review and improve clarity, flow, and formatting of `docs/capstone-project/index.md`
- [x] T-PREREQ-06 Create missing `docs/capstone-project/index.md` file
- [ ] T017 Convert long text into clean sections, bullet points, callouts where helpful in all reviewed `docs/` files
- [ ] T018 Fix any Markdown formatting issues in all reviewed `docs/` files

---

## Phase 4: Add Missing Content in Incomplete Files

- [ ] T019 Populate `docs/additional-materials/index.md` with relevant content (PDFs, external resources, cheatsheets, reference links)
- [ ] T020 Populate `docs/weekly-roadmap/index.md` with appropriate 8-13 week structure, weekly goals, learning milestones, and practice assignments

---

## Phase 5: Add Diagrams & Images in Modules

- [ ] T021 Add ROS 2 system architecture diagram to `static/img/diagrams/ros2_arch.png` (create image file)
- [ ] T022 Integrate ROS 2 system architecture diagram into `docs/module1-ros2-nervous-system/index.md`
- [ ] T023 Add digital-twin simulation pipeline diagram to `static/img/diagrams/digital_twin_pipeline.png` (create image file)
- [ ] T024 Integrate digital-twin simulation pipeline diagram into `docs/module2-digital-twin-simulation/index.md`
- [ ] T025 Add NVIDIA Isaac workflow diagram to `static/img/diagrams/isaac_workflow.png` (create image file)
- [ ] T026 Integrate NVIDIA Isaac workflow diagram into `docs/module3-ai-brain-nvidia-isaac/index.md`
- [ ] T027 Add VLA robotics flow diagram to `static/img/diagrams/vla_robotics_flow.png` (create image file)
- [ ] T028 Integrate VLA robotics flow diagram into `docs/module4-vision-language-action-robotics/index.md`

---

## Phase 6: Redesign `intro.md` (Book Front Page)

- [ ] T029 Add book cover mockup image to `static/img/book_cover.png` (create image file)
- [ ] T030 Add book cover, title, subtitle, and 1-2 paragraph description to `docs/Physical-AI-and-Humanoid-Robotics-Textbook/intro.md`
- [ ] T031 Create React component(s) for module cards (e.g., `src/components/ModuleCard.js`)
- [ ] T032 Integrate grid of clickable module cards into `docs/Physical-AI-and-Humanoid-Robotics-Textbook/intro.md`
- [ ] T033 Ensure each module card links directly to its respective module page

---

## Phase 7: Apply a Consistent Theme Across the Book

- [ ] T034 Define a single unified color theme (e.g., in `src/css/custom.css`)
- [ ] T035 Apply the chosen theme colors to callouts, headings, diagrams (if editable), cards, and buttons by modifying `src/css/custom.css` and `docusaurus.config.js`
- [ ] T036 Ensure aesthetic consistency in Markdown and Docusaurus configuration files

---

## Phase 8: Final QA Review

- [ ] T037 Check for broken links across all `docs/` content and fix them
- [ ] T038 Ensure all modules are reachable via sidebar and internal links
- [ ] T039 Re-check sidebar navigation flow for correctness and ease of use
- [ ] T040 Ensure all images load correctly by reviewing the site locally
- [ ] T041 Verify consistency in titles, typography, spacing, and formatting across the entire textbook
- [ ] T042 Build and preview the final Docusaurus site locally to ensure production quality

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Phase 1 (Sidebar)**: No dependencies.
-   **Phase 2 (Header/Footer)**: No dependencies.
-   **Phase 3 (Review Content)**: Can run in parallel with other content-focused phases.
-   **Phase 4 (Missing Content)**: Depends on content structure from Phase 3, can run in parallel with Phase 5.
-   **Phase 5 (Diagrams)**: Can run in parallel with Phase 4.
-   **Phase 6 (Intro Page)**: Depends on basic Docusaurus setup (implied from Phase 1/2).
-   **Phase 7 (Theme)**: Can run at any point, but most effective after core UI elements are in place.
-   **Phase 8 (QA)**: Depends on completion of all other implementation phases.

### Parallel Opportunities

-   Tasks within Phase 3 (reviewing individual module files) can be done in parallel.
-   Tasks within Phase 4 (populating different incomplete files) can be done in parallel.
-   Tasks within Phase 5 (adding diagrams to different modules) can be done in parallel.
-   Tasks in Phases 3, 4, 5, and 6 have potential for parallel execution by different team members, as they primarily involve distinct files or sections.

---

## Implementation Strategy

### Iterative Delivery

1.  Complete Phase 1 (Sidebar) and Phase 2 (Header & Footer) to establish core navigation and branding.
2.  Concurrently work on Phase 3 (Review Existing Content), Phase 4 (Add Missing Content), Phase 5 (Add Diagrams), and Phase 6 (Redesign Intro Page).
3.  Integrate Phase 7 (Theme) as early as possible to ensure visual consistency throughout the development process.
4.  Execute Phase 8 (Final QA) once all content and structural updates are complete.

---

## Notes

-   This task list is designed to be highly granular for efficient execution.
-   The `docs/` directory is the primary focus for content modifications.
-   `static/` will be used for images/assets.
-   `src/` and `docusaurus.config.js`, `sidebars.js` for configuration and custom components.
-   Regularly build and preview the Docusaurus site locally to check progress and identify issues early.