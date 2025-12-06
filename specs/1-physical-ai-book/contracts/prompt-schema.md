# Prompt Contract / Schema Definition

This document defines the expected structure and content for all prompt files stored in `history/prompts/`. Adherence to this schema is crucial for reproducibility (SC-004) and maintaining prompt transparency (Principle 3).

## 1. File Naming Convention

Prompt files MUST be named consistently to facilitate tracking and automation.

-   **Format**: `<ID>-<slug>.<stage>.prompt.md`
    -   `<ID>`: A unique, monotonically increasing integer identifier within its feature/module.
    -   `<slug>`: A short, descriptive, hyphen-separated string derived from the prompt's title.
    -   `<stage>`: The current development stage (e.g., `spec`, `plan`, `tasks`, `red`, `green`, `refactor`, `explainer`, `misc`, `general`, `constitution`).
    -   `.prompt.md`: Fixed suffix indicating a prompt file.

**Example**: `1001-define-spec-for-humanoid-robotics-book.spec.prompt.md`

## 2. YAML Frontmatter Structure

Every prompt file MUST begin with a YAML frontmatter block.

```yaml
---
id: <integer> # Unique ID for the prompt within its context
title: "<string>" # Concise title (3-7 words)
stage: <string> # One of: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general
date_iso: YYYY-MM-DD # ISO formatted date of prompt creation
model: "<string>" # The specific Gemini model used (e.g., "gemini-2.5-flash")
surface: "agent" # Indicates prompt originated from an agent
feature: "<string>" # The feature name this prompt belongs to (e.g., "1-physical-ai-book" or "none" if general)
branch: "<string>" # The git branch where the prompt was created
user: "<string>" # User associated with the prompt
command: "<string>" # The CLI command that generated this prompt
labels: ["<string>", ...] # Optional: array of labels for categorization (e.g., ["chapter-3", "initial-draft"])
links: # Optional: relevant external links
  spec: "<url_or_path>" # Link to related spec file
  ticket: "<url_or_path>" # Link to related issue/ticket
  adr: "<url_or_path>" # Link to related ADR
  pr: "<url_or_path>" # Link to related pull request
files_yaml: # Optional: List of files created/modified by the prompt (one per line)
  - "<path/to/file1.md>"
  - "<path/to/file2.py>"
tests_yaml: # Optional: List of tests run/added by the prompt (one per line)
  - "test_chapter_3_code.py"
prompt_text: |
  <Full, verbatim user input that led to this prompt>
response_text: |
  <Key assistant output, concise summary of action/result>
outcome: "<string>" # Overall outcome (e.g., "success", "partial", "failure")
evaluation: "<string>" # Brief evaluation of the prompt's effectiveness (e.g., "Good first draft", "Needs refinement")
---
```

## 3. Prompt Content (Markdown)

Following the YAML frontmatter, the remainder of the file contains the actual prompt text provided to the Gemini CLI. This content MUST be clear, specific, and adhere to the project's writing style and constraints.

### Key Content Elements for Chapter Generation Prompts:

-   **Target Chapter**: Clearly state which chapter is being generated (e.g., "Generate Chapter 03: ROS 2 Architecture Deep Dive").
-   **Context**: Provide context from the `spec.md` and `plan.md` relevant to the chapter.
-   **Previous Chapters**: Reference content from preceding chapters to maintain flow and consistency.
-   **Specific Requirements**: Include any FRs or NFRs that apply directly to the chapter (e.g., number of code snippets, specific diagrams, citation requirements).
-   **Style and Tone**: Reiterate expectations for writing style, tone, and Flesch-Kincaid level.
-   **Content Structure**: Outline the desired sections, subsections, and key topics to be covered within the chapter.
-   **Code Snippet Instructions**: Specify requirements for code (e.g., runnable, tested, simulation-only).
-   **Diagram Instructions**: Specify type (Mermaid/SVG), theme, and content for diagrams.
-   **Citation Guidance**: Remind Gemini about APA 7th edition and source quality (peer-reviewed, recent).
-   **Out-of-Scope**: Clearly state what should NOT be included in the chapter.

Adherence to this contract ensures that prompts are well-formed, traceable, and capable of driving the Gemini CLI to produce high-quality, reproducible content for the book.
