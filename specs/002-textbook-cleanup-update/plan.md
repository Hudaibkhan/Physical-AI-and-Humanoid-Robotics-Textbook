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
