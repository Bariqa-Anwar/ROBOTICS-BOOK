---
id: 1
title: Specify VLA Module
stage: spec
date_iso: 2025-12-06
surface: agent
model: gemini-1.5-flash
feature: 04-vla
branch: main
user: user
command: ''
labels:
  - specification
  - module
  - vla
  - conversational-robotics
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - history/prompts/04-vla/1-specify-vla-module.spec.prompt.md
tests_yaml: []
prompt_text: |
  # /sp.modules/04-vla.yml
  /sp.module 04 – Vision-Language-Action (VLA) & Conversational Robotics
  Weeks: 11–13 + Capstone
  Learning Outcomes:
    - Convert natural language commands into executable ROS 2 action sequences
    - Build end-to-end voice-controlled humanoid (simulation → real hardware)
    - Integrate Whisper + Gemini 2.5 Flash + ROS 2 in <200 ms latency

  Required Chapters:
    Chapter 12 – Vision-Language-Action Models – The New Robotics Paradigm
    Chapter 13 – Voice-to-Action Pipeline with OpenAI Whisper + Gemini 2.5 Flash
    Chapter 14 – LLM → ROS 2 Action Server Translation Layer
    Chapter 15 – Capstone Project – Full Autonomous Conversational Humanoid

  Word count: 12,000–15,000 words
  Minimum citations: 22
  Deliverables:
    - Complete “Clean the room” → navigation + grasping pipeline
    - One-click deploy script for Jetson Orin Nano
    - Video demo of simulated + real humanoid responding to live voice commands

  Success criteria:
    - Student issues voice command “Bring me the red cup from the kitchen”
    - Humanoid (sim or real) completes task autonomously in <3 minutes
    - Entire pipeline runs on the $700 Jetson student kit

  Constraints:
    - Must use only Gemini 2.5 Flash (or newer) as the cognitive brain
    - No internet dependency at inference time after initial model download
    - Full offline capability on Jetson
response_text: |
  Thank you for providing the detailed specification for the "04 - Vision-Language-Action (VLA) & Conversational Robotics" module. I have recorded this specification.

  I will now create a Prompt History Record for this input.
---
