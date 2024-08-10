
# Path Planning Algorithms

This repository contains implementations of various path planning algorithms using OpenCV and other techniques. The project showcases a range of algorithms, each developed and refined over a period of five weeks. The goal is to compare and analyze the effectiveness of each approach in finding optimal paths.

Overview

The project is organized into several folders, each corresponding to a specific week’s work. Here’s a brief overview of what each folder contains:

Week 1: OpenCV for Path Planning

In the first week, we focused on utilizing OpenCV to implement basic path planning techniques. This involved setting up the environment, preprocessing images, and performing initial pathfinding tasks using computer vision methods.

	•	Key Features:
	•	Image preprocessing
	•	Feature detection
	•	Basic pathfinding using OpenCV

Week 2: A* Algorithm Implementation

The second week was dedicated to implementing the A* (A-star) algorithm. A* is a popular pathfinding and graph traversal algorithm used extensively in various applications, from robotics to game development.

	•	Key Features:
	•	Heuristic-based pathfinding
	•	Node evaluation
	•	Path optimization

Week 3: Rapidly-exploring Random Tree (RRT)

In the third week, the focus shifted to the Rapidly-exploring Random Tree (RRT) algorithm. RRT is an algorithm designed for path planning in high-dimensional spaces, providing a way to efficiently explore and find paths in complex environments.

	•	Key Features:
	•	Random tree growth
	•	Path exploration in high-dimensional spaces
	•	Obstacles avoidance

Week 4: RRT* Algorithm Implementation

The fourth week involved enhancing the RRT algorithm to RRT*. RRT* is an improved version of RRT that optimizes the paths found, leading to more efficient and cost-effective solutions.

	•	Key Features:
	•	Path optimization
	•	Iterative refinement
	•	Improved path quality

Week 5: Neural A* Algorithm

The final week focused on implementing the Neural A* algorithm, which combines the A* algorithm with neural networks to enhance pathfinding capabilities. This approach leverages machine learning to improve heuristic accuracy and pathfinding efficiency.

	•	Key Features:
	•	Integration of neural networks with A*
	•	Enhanced heuristic estimation
	•	Improved pathfinding performance

Comparison and Analysis

The project includes a comparative analysis of all the implemented algorithms. Each week’s implementation is tested and evaluated based on various metrics such as path length, computation time, and resource utilization. The results provide insights into the strengths and weaknesses of each algorithm, aiding in the selection of the most suitable approach for different scenarios.

Getting Started

To get started with the project, clone the repository and follow the instructions in each folder. Ensure you have the necessary dependencies installed, which are listed in the respective folders.

` git clone https://github.com/nikhilasornapudi/Path-Planning-Algorithms.git `


Dependencies

	•	Python 3.x
	•	OpenCV
	•	NumPy
	•	Matplotlib (for visualization)
	•	TensorFlow/Keras (for Neural A*)

