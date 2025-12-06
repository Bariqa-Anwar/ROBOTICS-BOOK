---
id: 1
title: Specify Isaac Module
stage: spec
date_iso: 2025-12-06
surface: agent
model: gemini-1.5-flash
feature: 03-isaac
branch: main
user: user
command: ''
labels:
  - specification
  - module
  - isaac
  - humanoid-robotics
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - history/prompts/03-isaac/1-specify-isaac-module.spec.prompt.md
tests_yaml: []
prompt_text: |
  # /sp.modules/03-isaac.yml
  /sp.module 03 – The AI-Robot Brain (NVIDIA Isaac™)
  Weeks: 8–10
  Learning Outcomes:
    - Master NVIDIA Isaac Sim for synthetic data generation at scale
    - Deploy Isaac ROS (hardware-accelerated VSLAM, Nav2, perception) on Jetson
    - Achieve stable bipedal walking using pretrained policies + fine-tuning
    - Perform sim-to-real transfer with <15 cm localization error

  Required Chapters:
    Chapter 09 – Isaac Sim Deep Dive – Domains, Randomization, Synthetic Data
    Chapter 10 – Isaac ROS Stack – VSLAM, Nav2, Manipulation, CUDA-accelerated Perception
    Chapter 11 – Bipedal Locomotion, Zero-Moment Point, Whole-Body Control

  Word count: 11,000–13,000 words
  Minimum citations: 25 (heavy on NVIDIA technical blogs + Isaac papers)
  Code deliverables:
    - Full Isaac Sim humanoid domain randomization script
    - Working Nav2 + MoveIt2 setup for 22+ DoF humanoid
    - Sim-to-real deployment recipe for Jetson Orin

  Success criteria:
    - Student can generate 10,000 synthetic images with ground-truth segmentation in <2 hours
    - Simulated humanoid walks 50 m in cluttered apartment without falling
    - Real Jetson achieves >20 Hz VSLAM on RealSense D435i

  Constraints:
    - Must target Isaac Sim 2023.1.1 or newer
    - All examples must run on a single RTX 4070 Ti / 4080 laptop
response_text: |
  Thank you for providing the detailed specification for the "03 - The AI-Robot Brain (NVIDIA Isaac™)" module. I have recorded this specification.

  I will now create a Prompt History Record for this input.
---
