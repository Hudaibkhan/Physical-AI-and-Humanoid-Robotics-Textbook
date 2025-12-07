<!-- Sync Impact Report:
Version change: 1.0.0 -> 1.1.0
Modified principles:
  - [PRINCIPLE_1_NAME] -> Spec-Driven Writing
  - [PRINCIPLE_2_NAME] -> Technical Reliability
  - [PRINCIPLE_3_NAME] -> Beginner-Friendly + Professional Tone
  - [PRINCIPLE_4_NAME] -> Documentation Quality
  - [PRINCIPLE_5_NAME] -> Consistency
Added sections: Book Requirements, Technical Constraints
Removed sections: PRINCIPLE_6
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
  - .specify/templates/commands/*.md: ✅ updated
  - README.md: ⚠ pending (check for principle references)
Follow-up TODOs: RATIFICATION_DATE
-->
# AI/Spec-Driven Book Creation Constitution

## Core Principles

### I. Spec-Driven Writing
Every module, chapter, and section is initiated through specifications generated via Spec-Kit Plus.

### II. Technical Reliability
All explanations, examples, and tutorials must be technically accurate and verifiable.

### III. Beginner-Friendly + Professional Tone
Understandable for beginners but maintains professional documentation quality.

### IV. Documentation Quality
Writing style similar to high-quality open-source documentation (Docusaurus, Next.js, Python docs).

### V. Consistency
Terminology, formatting, and structure must remain consistent across all modules and chapters.

## Book Requirements

- Minimum **4 modules**
- Each module contains **3–5 chapters**
- Each chapter must include:
  - Introduction
  - Explanations / Tutorials
  - Code examples / Working demos
  - Best practices / Notes
  - References if applicable
- At least one real example must be tested, functional, and follow best practices.

### Structure Requirements
- Book hierarchy:
  - **Module → Chapters → Sections → Subsections**

### Documentation Style
- Use clear headings, tables, code blocks, and Mermaid diagrams.
- Include callouts: `:::info`, `:::note`, `:::tip`, `:::warning`

### Version Control
- All updates tracked and documented for GitHub Pages deployment.

### Writing Standards
- Clarity: Flesch-Kincaid readability score target: **Grade 9–12**
- Voice: Friendly, teacher-like tone, Highly structured and step-by-step
- Formatting: Markdown optimized for Docusaurus, Fenced code blocks with language tags, All content must be original (0% plagiarism tolerance)

## Technical Constraints

- **Platform:** Docusaurus v3 (unless updated)
- **Deployment:** GitHub Pages using GitHub Actions
- **Output:**
  - Fully working website
  - Published on GitHub Pages
  - All pages auto-generated using Spec-Kit workflow (specify → clarify → plan → tasks working sample project included per module)

### Constraints
- Every module and chapter must follow the Spec-Kit workflow:
  - `/sp.specify`
  - `/sp.clarify`
  - `/sp.plan`
  - `/sp.tasks`
  - `/sp.implement`
- Output only Markdown compatible with Docusaurus.
- Avoid AI hallucinations; verify all technical content.

## Governance

This constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All pull requests and reviews must verify compliance. Complexity must be justified.

### Success Criteria
- Book builds and runs locally in Docusaurus.
- GitHub Pages deployment succeeds.
- Consistent formatting and structure across all modules and chapters.
- All code examples work correctly.
- Spec-Kit workflow followed end-to-end.
- Final book is polished, clear, and professional.

**Version**: 1.1.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date unknown | **Last Amended**: 2025-12-04