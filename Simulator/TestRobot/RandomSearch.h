#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <ctime>
#include <math.h>
#include <algorithm>
#include <queue>
#include <map>
#include <set>
#include <windows.h>

#include "Point.h"
#include "Task.h"
#include "Map.h"
#include "Robot.h"
#include "CommonFunctions.h"
#include "Log.h"

#define WaitTime 500

using namespace std;

class RandomSearch {
public:
	// main function
	void Processing();

	// supporting functions
	void TaskExtension(Task* task, MatrixMap* map);
	
private:
};

void RandomSearch::Processing() {
	// read map, the origin is in the leftmost top,  x means rows, y means columns
	MatrixMap* world = new MatrixMap();
	world->ReadMap();
	world->Display("obstacle"); //world.Display();

	// read task, generate assembly tree
	Task* task = new Task();
	task->ReadTask();
	task->GenerateTree();
	//cout << endl << "Depth: " << task->AssemblyTree.depth(task->AssemblyTree.root()) << endl;
	//cout << endl << world->TaskCheck(1, task->AssemblyTree.leaves()[0]->data, 2) << endl; 

	// create the robots
	vector<Robot*> robot;
	for (int i = 0; i < task->robotNum; i++) {
		Robot* temp = new Robot(task->robotNum, task->startPoints[i]->id, task->startPoints[i]->taskPoint, world->RowNum, world->ColNum);
		robot.push_back(temp);
	}

	// Extend the task components, according to the assembly tree
	TaskExtension(task, world);

	// assign the task to the closest robots using optimization (or bid)
	AssignTaskToRobot(task, robot);
	// show the task extension process
	RecordTaskExtend(task, robot);

	// ID to index
	vector<vector<int>> idToIndex = IDtoIndex(robot);
	vector<int> tID2index = idToIndex[0];  // input: task ID  output: robot index
	//vector<int> rID2index = idToIndex[1];  // input: robot ID output: robot index
	
	// Robot movement
	//RobotMove(task, robot, world, tID2index);
	RobotMove_LocalPlan(task, robot, world, tID2index);
	//system("pause");
	Recover(task);
}

void RandomSearch::TaskExtension(Task* task, MatrixMap* map) {
	task->PushAll("allExtendedPoints");
	vector<int> sepStuckGroups;
	vector<int> moveStuckGroups;
	int depth = task->AssemblyTree.depth(task->AssemblyTree.root());
	for (int i = 0; i < depth - 1; i++) { // the leaf layer of assembly tree is not needed for extension
		// construct the task subgroups
		vector<TaskSubgroup>* taskGroups = new vector<TaskSubgroup>();
		GetTaskSubgroups(taskGroups, task->AssemblyTree.root(), task->SegTree.root(), task, 0, i);

		cout << endl << endl << "Depth proess:   " << i << endl
			<< "How many groups :  " << taskGroups->size() << endl;
		bool sepComplete = false;
		vector<int> collision;
		while (!sepComplete) {
			for (int i = 0; i < taskGroups->size(); ++i) {
				cout << endl
					<< "The leader ID " << i << " : \t" << (*taskGroups)[i].leader
					<< "\tSeparation distance: \t" << (*taskGroups)[i].targetSepDistance
					<< "\tCurrent distance: \t" << (*taskGroups)[i].currentSepDistance << endl;
				if (!(*taskGroups)[i].sepDone) {
					// normal separation
					(*taskGroups)[i].Separation(map);
					task->PushAll("allExtendedPoints");
					// if two sides are obstacle, shear separation

				}
			}
			// EndCheck: if all Done, complete; otherwise, move
			sepComplete = true;
			for (int i = 0; i < taskGroups->size(); ++i) {
				if (!(*taskGroups)[i].sepDone) { sepComplete = false; break; }
			}
			cout << "Separation complete? :  " << sepComplete << endl;
			// if separation is not complete, move. at most one side is obstacle. 
			if (!sepComplete) {
				// move
				for (int i = 0; i < taskGroups->size(); ++i) {
					(*taskGroups)[i].OverallMove(map);
				}
				//
				task->PushAll("allExtendedPoints");
				map->Display("task");
			}
		}

		// display
		cout << endl << "Extend step " << i << " : " << endl;
		map->Display("task");
		cout << endl;
		task->PushAll("allTargets");
	}
}
