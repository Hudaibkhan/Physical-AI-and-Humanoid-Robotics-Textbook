# Quickstart Guide: Physical AI Textbook Cleanup & Update

## Overview
This guide provides a quick reference for implementing the Physical AI & Humanoid Robotics textbook cleanup and update project.

## Prerequisites
- Node.js and npm installed
- Docusaurus v3 environment
- Git access to the repository
- Appropriate permissions to modify files and directories

## Implementation Steps

### 1. Directory Cleanup
```bash
# Delete outdated files
rm docs/final-exam.md
rm docs/glossary.md
rm docs/hardware.md
rm docs/overview.md

# Delete old module folders
rm -rf docs/module1-ros2-nervous-system
rm -rf docs/module2-digital-twin-simulation
rm -rf docs/module3-ai-brain-nvidia-isaac
rm -rf docs/module4-vision-language-action-robotics
rm -rf specs/001-book-content-update
```

### 2. Directory Renaming
```bash
# Rename module directories
mv docs/ros2-nervous-system docs/module1-ros2-nervous-system
mv docs/digital-twin-simulation docs/module2-digital-twin-simulation
mv docs/ai-brain-isaac docs/module3-ai-brain-isaac
mv docs/vla-robotics docs/module4-vla-robotics
```

### 3. Content Updates
Update the following files with the required content:
- `docs/intro.md` - Add book title, description, table of contents, module cards preview, hero banner
- `docs/weekly-roadmap.md` - Add complete 1-13 week roadmap with diagrams
- `docs/additional-materials/*` - Add missing content, explanations, examples, diagrams
- All module index pages - Add complete content with learning objectives, diagram placeholders, chapter summaries
- Update `sidebars.js` with new structure

### 4. Front Page Creation
Create `src/pages/index.tsx` with:
- Book title and description
- "Start Reading" button
- 4 module cards with icons, titles, descriptions, and links
- Consistent futuristic robotics theme

### 5. Theme Application
Update `src/css/custom.css` with the futuristic robotics theme featuring blue/purple neon accents.

## Verification Steps
1. Run `npm run build` to ensure Docusaurus builds without errors
2. Test all navigation links work correctly
3. Verify all module cards link to correct modules
4. Confirm sidebar follows specified structure
5. Check that all content displays properly with new theme

## Common Issues and Solutions
- Broken links after directory renaming: Update all internal links to reflect new paths
- Build errors: Check for missing files or incorrect paths
- Theme not applying: Verify CSS file paths and class names
- Sidebar not updating: Clear Docusaurus cache and rebuild