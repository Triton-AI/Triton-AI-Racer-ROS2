# Triton AI Racer ROS2 Contributing Guide

Thank you for your time in contributing to our autonomous racer. Please take a minute to review this contributing guide before you start.

## Getting Started

### Prerequisites

We generally recommend these steps as prerequisites before you start to contribute to the project.

- ROS2 development experience or completion of the [qualification task](https://github.com/Triton-AI/Triton-AI-Racer-ROS2/issues/1),
- Modern C++ and/or Python,
- Source control (git),
- Linux (Ubuntu) and Docker.

### Setting Up

Read the [README](README.md) for the basic info of the project.

Set up an X86 workstation with Ubuntu 20.04 (or VM), or WSL2 (with Docker Windows Desktop).

Install the following packages:

- [ROS2 Galactic](https://docs.ros.org/en/galactic/index.html),
- [Docker](https://docs.docker.com/get-docker/),
- Your favourite code editor (Visual Studio Code / CLion / Atom / Vim).

### Repository Structure

- `src`: all source files;
- `docs`: general usage documentations (package specific docs are stored in each package);
- `tools`: runtime and post-runtime utilities for data parsing, analyzing and visualization;
- `docker`: scripts and docker files for development and deployment.

### Git Practices

`main` branch is protected by one peer code review in the pull request.

To develop a new package, or work on a bug fix:

1. If you are a Triton AI member, ask to be added to the GitHub team and clone the repository. If not, fork it.
2. Checkout a new branch with naming convention like `tom/feature/new_pkg` or `jerry/bugfix/old_pkg`.
3. Work on your code. When you finish, rebase your code onto `main` to catch up with the changes.
4. Submit a pull request (PR). Assign your self to the PR and request review from one of your peer developers.
5. Communicate over the PR and LGTM.

### ROS2 Practices

1. Think twice before reinventing the wheel.
2. Create your package under the appropriate subfolder under `src`.
3. Use `param` instead of `config` folder for storing param files.
4. Have a `design.md` or a `design` folder that contains documentation explaining
    1. Purpose of your package,
    2. Publishers and subscribers,
    3. Configurable params,
    4. Services and actions,
    5. Inner-working, algorithm, etc.
5. Use `ament_cmake_auto` instead of `ament_cmake` to manage dependencies in`CMakeList.txt`.
6. Prioritize C++ over Python especially in production and real-time safety-critical nodes.
