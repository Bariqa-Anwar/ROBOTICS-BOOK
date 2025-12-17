# Data Model for "Physical AI and Humanoid Robotics Book"

This document defines the key entities and their relationships within the "Physical AI and Humanoid Robotics Book" project, as extracted from the feature specification.

## Entities

### Book

-   **Description**: The primary deliverable, a comprehensive technical book on Physical AI and Humanoid Robotics.
-   **Attributes**:
    -   `title`: "Physical AI and Humanoid Robotics"
    -   `author`: Google Gemini 2.5 Flash
    -   `total_word_count`: 45,000 - 60,000 words (FR-003)
    -   `number_of_chapters`: 14 (FR-002)
    -   `number_of_appendices`: 4 (FR-002)
    -   `citation_count`: Minimum 80, with ≥ 60% from peer-reviewed/official sources (FR-004)
    -   `publication_date`: Target February 28, 2026 (SC-001)
    -   `published_url`: Publicly accessible Docusaurus site on GitHub Pages (SC-001, FR-006)
-   **Relationships**: Composed of `Chapters` and `Appendices`. Utilizes `Prompts` for generation.

### Chapter

-   **Description**: A self-contained markdown (MDX) file covering a specific topic within the book.
-   **Attributes**:
    -   `title`: Unique title for the chapter.
    -   `filename`: `chXX-chapter-name.mdx` format.
    -   `content`: Prose, code snippets, diagrams, tables, captions.
    -   `citations`: References to external sources.
    -   `gemini_only_authorship`: TRUE (Principle 1, FR-001)
    -   `tested_code_examples`: TRUE (FR-005, FR-008)
    -   `style`: Authoritative yet accessible; Flesch-Kincaid Grade 10–12 (Constitution)
    -   `tone`: Optimistic but grounded, technically rigorous, forward-looking (Constitution)
-   **Relationships**: Belongs to `Book`. Contains `Code Snippets`, `Diagrams`, `Citations`. Generated from a `Prompt`.

### Code Snippet

-   **Description**: Runnable blocks of code (e.g., Python/ROS 2) demonstrating technical concepts.
-   **Attributes**:
    -   `language`: Python, ROS 2 specific (e.g., C++ for nodes)
    -   `runtime_environment`: Ubuntu 22.04, ROS 2 Humble (LTS), NVIDIA Isaac Sim 2023.1.1+ (FR-005)
    -   `functionality`: MUST be tested and functional (FR-005, FR-008)
    -   `status`: Zero broken code snippets (FR-008)
-   **Relationships**: Contained within `Chapters`.

### Diagram

-   **Description**: Visual explanations of architecture, processes, or concepts.
-   **Attributes**:
    -   `format`: Mermaid syntax or Gemini-generated SVG (Constitution, Key Entities)
    -   `theme`: Mermaid theme #00ff9d (Key Decisions Locked)
    -   `placement`: Embedded within `Chapters`.
-   **Relationships**: Contained within `Chapters`.

### Citation

-   **Description**: A reference to an external source, ensuring academic rigor and avoiding plagiarism.
-   **Attributes**:
    -   `format`: APA 7th edition (Constitution, Key Decisions Locked)
    -   `credibility`: Peer-reviewed papers or official NVIDIA/ROS documentation (FR-004)
    -   `date_range`: Primarily 2020–2025 (Acceptance Scenario, FR-004)
    -   `embedded_link`: Appropriately linked (Key Entities)
-   **Relationships**: Referenced by `Chapters`.

### Prompt

-   **Description**: Text input to the Gemini CLI used to generate book content.
-   **Attributes**:
    -   `content`: The prompt text itself.
    -   `frontmatter`: YAML metadata (e.g., chapter ID, date, status)
    -   `version_controlled`: TRUE (Principle 3)
    -   `reproducibility`: MUST allow for regeneration of content (SC-004)
-   **Relationships**: Generates `Chapters`. Stored in `history/prompts/`.
