---
id: 0002
title: Generate atomic tasks for AI/Spec-Driven Book
stage: tasks
date: 2025-12-10
surface: agent
model: unspecified
feature: AI/Spec-Driven Book: Physical AI & Humanoid Robotics
branch: master
user: unknown
command: /sp.tasks
labels: ["tasks", "task-generation", "atomic-tasks", "book-project"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/master/tasks.md
  - .specify/memory/constitution.md
  - specs/master/plan.md
  - .specify/templates/tasks-template.md
tests:
  - none
---

## Prompt

The provided text perfectly outlines the principles of the Speckitplus **Tasks Phase**—the critical need to break down the work into atomic (15-30 minute) units with clear dependencies and single acceptance criteria to enforce the **Checkpoint Pattern**.

Following these strict principles, the `/sp.tasks` command should not be a list of tasks written by you, but a high-level instruction asking the Agent to **generate the atomic tasks** based on the *Constitution* and the *Plan* we created previously.

Here is the high-level prompt for the `/sp.tasks` command, focusing on the four modules of your book and enforcing the checkpoint pattern.

```
/sp.tasks AI/Spec-Driven Book: Physical AI & Humanoid Robotics

Generate: Atomic task breakdown for the entire project, organized by the four core modules defined in the Constitution (Introduction/Setup, Module 1, Module 2, Module 3, Module 4).

Constraints:
- **Atomic Size:** Each task must be executable in 15-30 minutes.
- **Single Criterion:** Each task must have one clear, verifiable acceptance criterion (e.g., "Code snippet passes linting," "MDX file has 3+ sections").
- **Output Format:** All outputs must be Docusaurus-ready Markdown (MDX) files or verified code/config snippets.
- **Dependencies:** Clearly specify the Task ID dependency for each generated task.

Checkpoint Strategy:
- **Phase Checkpoint:** Enforce a mandatory human checkpoint after the completion of *each major Module* (i.e., after Introduction/Setup, after Module 1, after Module 2, after Module 3, and after Module 4).
- **Internal Checkpoints:** Place internal checkpoints within Modules 2 and 3 where research transitions to complex code implementation (e.g., after **Sensor Research** but before **ROS 2 Driver Implementation**).

Focus Breakdown (Targeting the four modules):
1.  **Introduction/Setup:** Focus on environment configuration and basic URDF creation.
2.  **Module 1 (Foundation):** Focus on basic ROS 2 rclpy node creation and Gazebo world integration.
3.  **Module 2 (Sensors & Perception):** Focus on integrating NVIDIA Isaac Sim/Vision and writing perception nodes.
4.  **Module 3 (Control & Action):** Focus on advanced control loops and inverse kinematics implementation.
5.  **Module 4 (HRI & VLA):** Focus on LLM (GPT) and VLA (Whisper) integration for human interaction.
```

## Response snapshot

The `tasks.md` file has been successfully generated, providing an atomic task breakdown for the "AI/Spec-Driven Book: Physical AI & Humanoid Robotics" project. The tasks are organized into five modules (Introduction/Setup, Module 1, Module 2, Module 3, Module 4), adhering to atomicity (15-30 minutes per task), single acceptance criteria, and specific output format. Checkpoints, both human and internal, are strategically placed within the task list. The file contains 68 tasks (T001-T068).

## Outcome

- ✅ Impact: Atomic task breakdown for the "AI/Spec-Driven Book: Physical AI & Humanoid Robotics" project generated, organized by modules with checkpoints and dependencies.
- 🧪 Tests: none
- 📁 Files: Created `specs/master/tasks.md` with 68 atomic tasks.
- 🔁 Next prompts: Proceed with implementing the tasks, starting with Module 0.
- 🧠 Reflection: Consistent failure of bash scripts in a PowerShell environment highlights the need for environment-aware scripting.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script failed because it was a bash script and the environment was PowerShell.
- Graders run and results (PASS/FAIL): None
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): Implement platform-aware script execution or provide PowerShell alternatives for core scripts.