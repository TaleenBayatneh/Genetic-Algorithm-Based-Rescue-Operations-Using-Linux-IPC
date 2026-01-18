# Genetic-Algorithm-Based-Rescue-Operations-Using-Linux-IPC

# Project Overview

This project is developed as part of the **Real-Time Applications & Embedded Systems** course.

The main goal of the project is to simulate **rescue operations in a collapsed building** using:

- Genetic Algorithms (GA)
- Multi-processing
- Linux Interprocess Communication (IPC)

Instead of relying on a single robot or a single process, the system coordinates **multiple robots running as separate processes** to efficiently rescue survivors.

---

## Main Idea (Simple Explanation)

- The environment is modeled as a **3D grid** representing a collapsed building.
- Each robot is implemented as an **independent Linux process**.
- Robots search for survivors and deliver basic supplies.
- A **Genetic Algorithm** is used to optimize robot paths.
- Robots communicate using **IPC** to avoid conflicts and duplicated rescues.

This design demonstrates real-time coordination, optimization, and parallel execution.

---

## Genetic Algorithm Concept

- Each chromosome represents a **robot path** in 3D space.
- The population contains multiple possible paths.
- The GA improves paths over generations using:
  - Selection
  - Crossover
  - Mutation
  - Elitism

The goal is to maximize rescue efficiency while minimizing distance and risk.

---

## Fitness Function (Concept)

The fitness function evaluates paths based on:
- Number of survivors reached
- Area coverage
- Path length
- Risk level of visited cells

Better paths receive higher fitness scores and are more likely to evolve.

---

## Multi-Robot & IPC Concept

- Each robot runs as a **separate process**.
- Robots communicate using Linux IPC mechanisms such as:
  - Shared Memory
  - Semaphores
  - Message Queues

IPC is used to:
- Share survivor discovery information
- Prevent multiple robots from rescuing the same survivor
- Synchronize rescue operations

---

## A* Algorithm Usage

- A* is used to generate **initial feasible paths**.
- These paths serve as seeds for the Genetic Algorithm.
- A* is also used for comparison with GA results.

---

## Parallelism Concept

- Multiple robots run in parallel as separate processes.
- Each robot explores a different path.
- Parallel execution improves rescue time and coverage.

---

## Performance Evaluation

The system evaluates performance based on:
- Total path length
- Total rescue time
- Number of survivors rescued

Results from **GA-based multi-robot execution** are compared against **single-robot A\*** execution.

---

## Technologies Used

- C Programming Language
- Linux System Calls
- fork(), shared memory, semaphores
- Genetic Algorithms
- A* Pathfinding

---

## Notes

- Each robot can rescue only one survivor.
- The environment is static during execution.
- The project focuses on correctness, coordination, and optimization.
