# FSB Document Templates

This directory contains standard academic templates for the Faculty of Mechanical Engineering and Naval Architecture (FSB), University of Zagreb.

## Available Templates

| Template | Directory | Primary Format | Backup Format |
|----------|-----------|----------------|---------------|
| **Seminar** | `docs/templates/fsb-seminar/` | LaTeX (`.tex`) | Word (`.docx`) |
| **Thesis** | `docs/templates/fsb-thesis/` | LaTeX (`.tex`) | Word (`.docx`) |
| **Paper** | `docs/templates/fsb-paper/` | LaTeX (`.tex`) | Word (`.docx`) |

## Workflow for AI Agents

AI agents are instructed to generate content in **Markdown** format first. This Markdown content is then used to populate the templates.

### 1. Generate Content (Markdown)
Agents should follow the structure defined in `docs/templates/fsb-seminar/structure.md`.

### 2. Export to LaTeX (PDF)
Copy the generated Markdown sections into the `{{GENERIRANI_SADRZAJ}}` placeholder in the respective `.tex` file and compile using a LaTeX engine (like `pdflatex`).

### 3. Backup to Word
For quick edits or sharing with mentors who prefer Word, use the `.docx` templates in the `word/` subdirectories. You can use tools like `pandoc` to convert Markdown to Word using these templates as a reference.

## Template Files Breakdown

- `latex/`: Contains the `.tex` files for institutional styling.
- `word/`: Contains `.docx` and `.dotx` backup templates.
- `structure.md`: Defines the header mapping and document organization.
- `instructions.md`: Global instructions for AI agents on how to write for these templates.
