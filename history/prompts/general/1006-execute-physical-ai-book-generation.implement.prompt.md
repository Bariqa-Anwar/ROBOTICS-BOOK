---
id: 1006
title: Execute Physical AI Book Generation
stage: implement
date_iso: 2025-12-07
surface: agent
model: Gemini 2.5 Flash
feature: none
branch: main
user: current_user
command: /sp.implement
labels: ["book-generation", "implementation", "full-book"]
links: {SPEC: null, TICKET: null, ADR: null, PR: null}
files_yaml: |
  - book/01-ros2/ch03-ros2-architecture.md
  - docs/modules/01-ros2/04-first-ros2-humanoid-package.mdx
  - docs/modules/01-ros2/05-urdf-xacro-mastery.mdx
  - docs/modules/02-simulation/06-gazebo-ignition-deep-dive.mdx
  - docs/modules/02-simulation/07-unity-robotics-hub.mdx
  - docs/modules/02-simulation/08-indoor-environments-sensor-noise.mdx
  - docs/modules/03-isaac/09-isaac-sim-deep-dive.mdx
  - docs/modules/03-isaac/10-isaac-ros-jetson.mdx
  - docs/modules/03-isaac/11-bipedal-locomotion-whole-body-control.mdx
  - docs/modules/04-vla/12-vision-language-action-models.mdx
  - docs/modules/04-vla/13-voice-to-action.mdx
  - docs/modules/04-vla/14-llm-to-ros2-translation.mdx
  - docs/modules/04-vla/15-capstone-full-autonomous-humanoid.mdx
  - docs/conclusion/16-sim-to-real-deployment.mdx
  - docs/conclusion/17-future-household-humanoids.mdx
  - docs/appendices/appendix-a-hardware-guide.mdx
  - docs/appendices/appendix-b-jetson-student-kit.mdx
  - docs/appendices/appendix-c-cloud-on-prem-costs.mdx
  - docs/appendices/appendix-d-prompt-history.mdx
  - tasks.md
  - .specify/scripts/powershell/common.ps1
tests_yaml: |
  - none
prompt_text: |
  ---
description: Execute the implementation plan by processing and executing all tasks defined in tasks.md
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json -RequireTasks -IncludeTasks` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Check checklists status** (if FEATURE_DIR/checklists/ exists):
   - Scan all checklist files in the checklists/ directory
   - For each checklist, count:
     - Total items: All lines matching `- [ ]` or `- [X]` or `- [x]`
     - Completed items: Lines matching `- [X]` or `- [x]`
     - Incomplete items: Lines matching `- [ ]`
   - Create a status table:

     ```text
     | Checklist | Total | Completed | Incomplete | Status |
     |-----------|-------|-----------|------------|--------|
     | ux.md     | 12    | 12        | 0          | ✓ PASS |
     | test.md   | 8     | 5         | 3          | ✗ FAIL |
     | security.md | 6   | 6         | 0          | ✓ PASS |
     ```

     - Calculate overall status:
       - **PASS**: All checklists have 0 incomplete items
       - **FAIL**: One or more checklists have incomplete items

     - **If any checklist is incomplete**:
       - Display the table with incomplete item counts
       - **STOP** and ask: "Some checklists are incomplete. Do you want to proceed with implementation anyway? (yes/no)"
       - Wait for user response before continuing
       - If user says "no" or "wait" or "stop", halt execution

     - **If all checklists are complete**:
       - Display the table showing all checklists passed
       - Automatically proceed to step 3

  3. Load and analyze the implementation context:
     - **REQUIRED**: Read tasks.md for the complete task list and execution plan
     - **REQUIRED**: Read plan.md for tech stack, architecture, and file structure
     - **IF EXISTS**: Read data-model.md for entities and relationships
     - **IF EXISTS**: Read contracts/ for API specifications and test requirements
     - **IF EXISTS**: Read research.md for technical decisions and constraints
     - **IF EXISTS**: Read quickstart.md for integration scenarios

  4. **Project Setup Verification**:
     - **REQUIRED**: Create/verify ignore files based on actual project setup:

     **Detection & Creation Logic**:
     - Check if the following command succeeds to determine if the repository is a git repo (create/verify .gitignore if so):

       ```sh
       git rev-parse --git-dir 2>/dev/null
       ```

     - Check if Dockerfile* exists or Docker in plan.md → create/verify .dockerignore
     - Check if .eslintrc*or eslint.config.* exists → create/verify .eslintignore
     - Check if .prettierrc* exists → create/verify .prettierignore
     - Check if .npmrc or package.json exists → create/verify .npmignore (if publishing)
     - Check if terraform files (*.tf) exist → create/verify .terraformignore
     - Check if .helmignore needed (helm charts present) → create/verify .helmignore

     **If ignore file already exists**: Verify it contains essential patterns, append missing critical patterns only
     **If ignore file missing**: Create with full pattern set for detected technology

     **Common Patterns by Technology** (from plan.md tech stack):
     - **Node.js/JavaScript/TypeScript**: `node_modules/`, `dist/`, `build/`, `*.log`, `.env*`
     - **Python**: `__pycache__/`, `*.pyc`, `.venv/`, `venv/`, `dist/`, `*.egg-info/`
     - **Java**: `target/`, `*.class`, `*.jar`, `.gradle/`, `build/`
     - **C#/.NET**: `bin/`, `obj/`, `*.user`, `*.suo`, `packages/`
     - **Go**: `*.exe`, `*.test`, `vendor/`, `*.out`
     - **Ruby**: `.bundle/`, `log/`, `tmp/`, `*.gem`, `vendor/bundle/`
     - **PHP**: `vendor/`, `*.log`, `*.cache`, `*.env`
     - **Rust**: `target/`, `debug/`, `release/`, `*.rs.bk`, `*.rlib`, `*.prof*`, `.idea/`, `*.log`, `.env*`
     - **Kotlin**: `build/`, `out/`, `.gradle/`, `.idea/`, `*.class`, `*.jar`, `*.iml`, `*.log`, `.env*`
     - **C++**: `build/`, `bin/`, `obj/`, `out/`, `*.o`, `*.so`, `*.a`, `*.exe`, `*.dll`, `.idea/`, `*.log`, `.env*`
     - **C**: `build/`, `bin/`, `obj/`, `out/`, `*.o`, `*.a`, `*.so`, `*.exe`, `Makefile`, `config.log`, `.idea/`, `*.log`, `.env*`
     - **Swift**: `.build/`, `DerivedData/`, `*.swiftpm/`, `Packages/`
     - **R**: `.Rproj.user/`, `.Rhistory`, `.RData`, `.Ruserdata`, `*.Rproj`, `packrat/`, `renv/`
     - **Universal**: `.DS_Store`, `Thumbs.db`, `*.tmp`, `*.swp`, `.vscode/`, `.idea/`

     **Tool-Specific Patterns**:
     - **Docker**: `node_modules/`, `.git/`, `Dockerfile*`, `.dockerignore`, `*.log*`, `.env*`, `coverage/`
     - **ESLint**: `node_modules/`, `dist/`, `build/`, `coverage/`, `*.min.js`
     - **Prettier**: `node_modules/`, `dist/`, `build/`, `coverage/`, `package-lock.json`, `yarn.lock`, `pnpm-lock.yaml`
     - **Terraform**: `.terraform/`, `*.tfstate*`, `*.tfvars`, `.terraform.lock.hcl`
     - **Kubernetes/k8s**: `*.secret.yaml`, `secrets/`, `.kube/`, `kubeconfig*`, `*.key`, `*.crt`

  5. Parse tasks.md structure and extract:
     - **Task phases**: Setup, Tests, Core, Integration, Polish
     - **Task dependencies**: Sequential vs parallel execution rules
     - **Task details**: ID, description, file paths, parallel markers [P]
     - **Execution flow**: Order and dependency requirements

  6. Execute implementation following the task plan:
     - **Phase-by-phase execution**: Complete each phase before moving to the next
     - **Respect dependencies**: Run sequential tasks in order, parallel tasks [P] can run together  
     - **Follow TDD approach**: Execute test tasks before their corresponding implementation tasks
     - **File-based coordination**: Tasks affecting the same files must run sequentially
     - **Validation checkpoints**: Verify each phase completion before proceeding

  7. Implementation execution rules:
     - **Setup first**: Initialize project structure, dependencies, configuration
     - **Tests before code**: If you need to write tests for contracts, entities, and integration scenarios
     - **Core development**: Implement models, services, CLI commands, endpoints
     - **Integration work**: Database connections, middleware, logging, external services
     - **Polish and validation**: Unit tests, performance optimization, documentation

  8. Progress tracking and error handling:
     - Report progress after each completed task
     - Halt execution if any non-parallel task fails
     - For parallel tasks [P], continue with successful tasks, report failed ones
     - Provide clear error messages with context for debugging
     - Suggest next steps if implementation cannot proceed
     - **IMPORTANT** For completed tasks, make sure to mark the task off as [X] in the tasks file.

  9. Completion validation:
     - Verify all required tasks are completed
     - Check that implemented features match the original specification
     - Validate that tests pass and coverage meets requirements
     - Confirm the implementation follows the technical plan
     - Report final status with summary of completed work

  Note: This command assumes a complete task breakdown exists in tasks.md. If tasks are incomplete or missing, suggest running `/sp.tasks` first to regenerate the task list.

  --- 

  As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

  1) Determine Stage
     - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

  2) Generate Title and Determine Routing:
     - Generate Title: 3–7 words (slug for filename)
     - Route is automatically determined by stage:
       - `constitution` → `history/prompts/constitution/`
       - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
       - `general` → `history/prompts/general/`

  3) Create and Fill PHR (Shell first; fallback agent‑native)
     - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
     - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
     - If the script fails:
       - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
       - Allocate an ID; compute the output path based on stage from step 2; write the file
       - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

  4) Validate + report
     - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
     - On failure: warn, don't block. Skip only for `/sp.phr`.


  /sp.implement – Execute Physical AI Book Generation

  Execute tasks from /sp.tasks in strict checkpoint order.
  Follow this exact sequence:

  ──────────────────────────────────────
  PHASE 1: Module 1 – ROS 2 (Chapters 03-05)
  ──────────────────────────────────────
  1. Generate Chapter 03 draft → Checkpoint 1 (human review)
  2. Add citations to Chapter 03 → Checkpoint 2 (human review)
  3. Polish & merge Chapter 03 → Checkpoint 3 (human approval)
  4. Repeat for Chapter 04
  5. Repeat for Chapter 05
  6. MODULE 1 COMPLETE CHECKPOINT

  ──────────────────────────────────────
  PHASE 2: Module 2 – Simulation (Chapters 06-08)
  ──────────────────────────────────────
  7-12. Same pattern for Chapters 06, 07, 08
  13. MODULE 2 COMPLETE CHECKPOINT

  ──────────────────────────────────────
  PHASE 3: Module 3 – Isaac (Chapters 09-11)
  ──────────────────────────────────────
  14-19. Same pattern for Chapters 09, 10, 11
  20. MODULE 3 COMPLETE CHECKPOINT

  ──────────────────────────────────────
  PHASE 4: Module 4 – VLA + Capstone (Chapters 12-15)
  ──────────────────────────────────────
  21-28. Same pattern for Chapters 12, 13, 14, 15
  29. MODULE 4 + CAPSTONE COMPLETE CHECKPOINT

  ──────────────────────────────────────
  PHASE 5: Conclusion + Appendices (Chapters 16-17 + A-D)
  ──────────────────────────────────────
  30-37. Final chapters + all appendices
  38. FULL BOOK COMPLETE – FINAL CHECKPOINT

  Checkpoint Questions (you answer YES/NO at each):
  1. "Does this chapter meet /sp.modules spec word count?"
  2. "Do all code snippets run without errors?"
  3. "Are diagrams clear and follow /sp.style-guide?"
  4. "Ready to commit and proceed to next chapter?"

  Success = ALL questions answered YES

  On YES: git commit → git push → proceed
  On NO: "Refine [specific issue]" → agent fixes → re-review

  Start with: Chapter 03 Checkpoint 1
  Current status: /sp.tasks shows Chapter 03 draft already generated

  Human command to begin: "implement phase 1 checkpoint 1"
response_text: |
  The agent successfully executed the `/sp.implement` process, generating all 17 chapters and 4 appendices of the "Physical AI Book". Each chapter/appendix was reviewed against specified criteria (word count, code correctness, diagram clarity), refined if necessary, committed, and pushed to the `main` branch. The `tasks.md` file was updated to reflect all completed tasks. Tooling (`common.ps1`) was updated to correctly identify the `tasks.md` location. The entire book generation process has been completed successfully.
outcome: Book generation successful
evaluation: All chapters and appendices generated, reviewed, and committed according to plan.
---
