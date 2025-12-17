# Feature Specification: Physical AI Book (4-Module Course Edition)

**Feature Branch**: `2-refine-book-structure`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description specifying a mandatory 4-module structure for the book.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Progression Through Modules (Priority: P1)

As a university student, I want to follow the book's clear 4-module structure, so that my learning path aligns perfectly with my 13-week course syllabus, from understanding the 'nervous system' (ROS 2) to building a full 'brain' (VLA models).

**Why this priority**: This is the core pedagogical value proposition. The book's structure must match the course structure for it to be a successful textbook.

**Independent Test**: A student can complete the chapters and exercises within each module and achieve the specific learning outcomes defined for that module.

**Acceptance Scenarios**:

1.  **Given** completion of Module 1, **When** the user runs the created ROS 2 packages, **Then** they can control a basic URDF robot model and inspect its topics, services, and actions.
2.  **Given** completion of Module 3, **When** the user works with Isaac Sim and Isaac ROS, **Then** they can successfully run hardware-accelerated perception and control a simulated robot's balance.
3.  **Given** completion of Module 4, **When** the user runs the final capstone project, **Then** the simulated humanoid can correctly interpret and execute a spoken natural language command (e.g., "go to the table and pick up the apple").

---

### User Story 2 - Educator Course Adoption (Priority: P2)

As an educator, I want the Docusaurus site's navigation to be organized by the four main modules, so I can easily assign readings and projects that map directly to my weekly syllabus.

**Why this priority**: A clear, module-based structure is critical for educators to adopt the book as a course textbook.

**Independent Test**: An educator inspecting the live Docusaurus site can immediately see the four top-level collapsible modules in the sidebar navigation.

**Acceptance Scenarios**:

1.  **Given** the live book website, **When** an educator views the sidebar, **Then** "Module 1 – The Robotic Nervous System (ROS 2)", "Module 2 – The Digital Twin", "Module 3 – The AI-Robot Brain", and "Module 4 – Vision-Language-Action (VLA) Models" are visible as top-level categories.

---

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The book's content and the Docusaurus sidebar navigation MUST be explicitly organized into the four specified modules.
-   **FR-002**: Each of the four modules MUST begin with a dedicated overview page summarizing its learning outcomes and weekly syllabus breakdown.
-   **FR-003**: The book's content MUST be structured into the 17 chapters and 4 appendices as per the provided table of contents.
-   **FR-004**: The total word count MUST be between 50,000 and 65,000 words.
-   **FR-005**: The book MUST cite a minimum of 90 sources, with at least 65% from peer-reviewed papers or official documentation (2020–2025).
-   **FR-006**: The entire project MUST be authored by the Gemini 2.5 Flash model via the `gemini` CLI.
-   **FR-007**: All other technical guarantees, constraints, and out-of-scope items from the previous specification version remain in effect.

### Key Entities

-   **Module**: A top-level thematic section of the book (e.g., "The Robotic Nervous System"). Contains a group of related chapters.
-   **Chapter**: A self-contained markdown (MDX) file belonging to exactly one Module.
-   (Other entities like Book, Code Snippet, Diagram, Citation, and Prompt remain the same as the previous version).

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: The final Docusaurus site's primary navigation is built around the four-module structure, verified by visual inspection.
-   **SC-002**: A survey of pilot course students shows that at least 90% agree or strongly agree that the book's modular structure was easy to follow and aligned with their course's weekly topics.
-   **SC-003**: The project is not considered a success if the final book and website do not clearly and strictly reflect the original 4-module course structure.
-   (All other success criteria from the previous specification, such as Lighthouse scores, technical accuracy, and reproducibility, still apply).
