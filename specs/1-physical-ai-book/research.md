# Research Strategy for "Physical AI and Humanoid Robotics Book"

This document outlines the research approach to be followed during the content generation phase of the "Physical AI and Humanoid Robotics Book" project.

## Research Approach: Research-Concurrent Strategy

**Decision**: A "research everything first" strategy was rejected due to its potential slowness and the dynamic nature of content generation. A "Research-Concurrent" strategy has been adopted instead.

**Chosen Strategy**: Implement a Write → Research → Refine loop per chapter. This approach ensures that research is targeted and integrated iteratively.

### Process per Chapter:

1.  **Gemini 2.5 Flash writes first draft**: Gemini generates the initial draft of a chapter based on the spec and its existing knowledge.
2.  **Human runs targeted search**: The human Spec Owner/Prompt Engineer conducts targeted research using sources such as Google Scholar, arXiv, ROS Discourse, and official NVIDIA documentation.
3.  **Human curates sources**: The human then pastes 8–15 of the best, most relevant sources into a dedicated `/research/chapter-XX.md` file (e.g., `specs/1-physical-ai-book/research/chapter-03.md`).
4.  **Gemini rewrites chapter with citations**: Gemini then rewrites the chapter, incorporating the provided sources with perfect APA citations.

**Trade-off Accepted**: This iterative process is acknowledged to be slightly slower than a pure generation-only approach. However, this trade-off is accepted because it guarantees 80+ high-quality citations, enhances technical accuracy, and minimizes hallucinations, which are critical for a "professional-grade" book.
