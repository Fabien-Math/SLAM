# Multi-Robot Cave Exploration  

## Overview  

This project focuses on the autonomous exploration of underground cavities using a multi-robot system. Due to the inaccessibility of such environments, robotic exploration presents unique challenges, including communication difficulties, trajectory planning in confined spaces, and navigating steep inclines. The research aims to develop advanced exploration algorithms that enable autonomous coordination between robots without reliance on external control.  

## Key Features  

- **Decentralized Multi-Robot Communication:** A novel algorithm facilitates inter-robot communication without a central master, enhancing adaptability in real-world scenarios.  
- **Trajectory Planning:** A new method based on the successive division of the shortest path enables dynamic and efficient navigation.  
- **Master Robot Coordination:** An alternative approach leveraging a designated master robot ensures low data transfer and basic error management using the ARQ method.  
- **Python-Based Simulator:** A custom simulation environment allows real-time testing of communication protocols and trajectory planning strategies.  

## Motivation  

Caves and underground environments remain among the least explored places on Earth, offering potential discoveries in geology and biology. This project contributes to autonomous robotic exploration, making it possible to investigate such hidden environments without direct human intervention.  

## Future Work  

- Extending trajectory optimization for rugged and uneven terrains.  
- Enhancing communication protocols for greater robustness.  
- Implementing real-world testing on physical robotic platforms.  

This repository contains all the source code and simulation tools developed as part of this project.