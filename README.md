# The SAPOA Algorithm for Multi-agent System in Environments with Obstacles

## 1. Problem Description

​	This project implements the SAPOA algorithm in paper, *An Efficient Parallel Self-assembly Planning Algorithm for Modular Robots in Environments with Obstacles*. The algorithm is to solve the self-assembly planning problem for the multi-agent system.  

**1.1 The input:** 

​	The initial and target locations of robots and the initial map including the obstacles are given.

**1.2 The objective:** 

​	To minimize the overall moving steps for all robots.

**1.3 The constraint:** 

​	The robots are supposed to avoid collision with other robots as well as obstacles during the movement.

**1.4 Three assumptions are listed as follows:**

​	1) The modular robots only move up, down, left or right at a velocity of one grid per time step, without any rotation.

​	2) Once next to each other, the robots will be grouped by docking.  The assembled robots cannot be separated till the end.

​	3) The docking action is only conducted between two groups of robots, called 1-to-1 docking, due to the environmental disturbance.

## 2. Algorithm Description

**2.1 SAPOA algorithm**

​	The algorithm includes four stages, namely, assembly tree generation, target extension, dispatching and robot navigation. The algorithm details are described in the paper. 

**2.2 Other algorithms**

​	Naive algorithm

​	Adapted PAA algorithm

​	SAPOAnop algorithm

​	SAPOAads algorithm

## 3. How to run the code

**3.1 Preparation**

​	1) This algorithm is implemented in C++ in Visual Studio 2019. 

​	2) Install the eigen library in VS.

​	3) Download the code and open the ./Multi-Agent-System-Multi-Thread-OOP-20181105.sln project in VS.

**3.2 Code file architecture**

​	The lowest level，
​		/Lib directory
​		Point.h provides the point data structure.
​		BinTree.h provides the binary tree data structure.
​		Map.h provides the map data structure.

​	The second level，
​		Task.h
​		Robot.h

​	The third level，

​		NaiveAlg.h

​		VijayAlg.h

​		RandomNoPair.h

​		RandomSearch.h

​		SAPOAads.h

​		Hungarian.h

​		CommonFunction.h

​	The fourth level,
​		Main function  // Statistics and the comparison

**3.3 Steps**

​	1) Open the main.cpp 

​	2) Choose 1 or 0 in **vector<int> alg = { 0,0,0,1,0 }; ** to run the algorithms respectively for Naive, SAPOAnop, SAPOA, SAPOAads and APAA. 

​	3) Change the value in **int data_start = 1; int data_end = 26;**  to choose the data to be run.

​	4) Click Run.


