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

using namespace std;

class RandomNoPair {
public:
	// main function
	void Processing(string data_dir);

	// supporting functions
	bool TaskExtension(Task* task, MatrixMap* map);
	void PrepareTaskSubgroups(vector<TaskSubgroup>* taskGroups, BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, int depth, int obj);
	bool EndCheck(vector<TaskSubgroup>* taskGroups, int range);
	int GroupDistance(TaskSubgroup group1, TaskSubgroup group2);
	
	// variables
	int taskStep = 0;
	int robotStep = 0;
	int taskStep_sys = 0;
	int robotStep_sys = 0;
	bool isComplete = true;
private:
};


void RandomNoPair::Processing(string data_dir) {
	// read map, the origin is in the leftmost top,  x means rows, y means columns
	MatrixMap* world = new MatrixMap();
	world->ReadMap(data_dir);
	world->Display("obstacle"); //world.Display();

	// read task, generate assembly tree
	Task* task = new Task();
	task->ReadTask(data_dir);
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
	isComplete = TaskExtension(task, world);
	if (!isComplete) return;

	// assign the task to the closest robots using optimization (or bid)
	AssignTaskToRobot(task, robot);
	// show the task extension process
	RecordTaskExtend(task, robot);

	// Robot movement
	isComplete = RobotMove_LocalPlan(task, robot, world);
	if (!isComplete) return;
	
	//system("pause");
	Recover(task);
	
	//record the step
	vector<int> steps = RecordStep(task, robot);
	taskStep = steps[0];
	robotStep = steps[1];
}

bool RandomNoPair::TaskExtension(Task* task, MatrixMap* map) {
	task->PushAll("allExtendedPoints");

	vector<int> moveStuckGroups;
	int depth = task->AssemblyTree.depth(task->AssemblyTree.root());
	for (int i = 1; i < depth; i++) { // the leaf layer of assembly tree is not needed for extension
		// construct the task subgroups
		vector<TaskSubgroup>* taskGroups = new vector<TaskSubgroup>();
		PrepareTaskSubgroups(taskGroups, task->AssemblyTree.root(), task->SegTree.root(), task, 0, i);

		cout << endl << endl << "Depth proess:   " << i << endl
			<< "How many groups :  " << taskGroups->size() << endl;

		bool Complete = false;
		vector<int> collision;
		int repeatStep = 0;
		int deadLoop = 0;
		while (!Complete) {
			// move 
			if (!Complete) {
				// move
				for (int j = 0; j < taskGroups->size(); ++j) {
					(*taskGroups)[j].OverallMove(map);
				}
				//
				task->PushAll("allExtendedPoints");
				map->Display("task");
			}

			// EndCheck: if all Done, complete; otherwise, move
			Complete = EndCheck(taskGroups, 3);
			cout << "Separation complete? :  " << Complete << endl;

			// Fail check
			deadLoop++;
			if (deadLoop > DEADLOOP) {
				Recover(task);
				cout << endl << endl << "Error: System failed!!!" << endl;
				return false;
			}

			CheckFail(task) ? repeatStep++ : repeatStep = 0;
			if (repeatStep > REPEAT) {
				return false;
			}
		}

		// display
		cout << endl << "Extend step " << i << " : " << endl;
		map->Display("task");
		cout << endl;
		task->PushAll("allTargets");
	}
	return true;
}

// prepare the task groups
void RandomNoPair::PrepareTaskSubgroups(vector<TaskSubgroup>* taskGroups, BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, int depth, int obj) {
	if (!assNode) return;   // tree node empty
	if (assNode->data.size() < 1) return;  // cannot be extended anymore

	if (depth == obj) {
		// construct the taskgroups
		//vector<int> lcomponents = assNode->lChild->data;
		//vector<int> rcomponents = assNode->rChild->data;

		vector<int> lcomponents = assNode->data;

		vector<TaskPoint*> ltask;
		vector<TaskPoint*> rtask;
		for (int i = 0; i < task->currentTargets.size(); ++i) {
			// lcomponents
			if (ltask.size() < lcomponents.size()) {
				for (int j = 0; j < lcomponents.size(); ++j)
					if (task->currentTargets[i]->id == lcomponents[j]) {
						ltask.push_back(task->currentTargets[i]);
						break;
					}
			}
		}
		// initialization
		TaskSubgroup taskgroup(ltask, rtask, segNode->data, 3);  // range = 3
		taskGroups->push_back(taskgroup);
	}

	PrepareTaskSubgroups(taskGroups, assNode->lChild, segNode->lChild, task, depth + 1, obj);
	PrepareTaskSubgroups(taskGroups, assNode->rChild, segNode->rChild, task, depth + 1, obj);
}

bool RandomNoPair::EndCheck(vector<TaskSubgroup>* taskGroups, int range) {
	bool endflag = true;
	int size = taskGroups->size();

	for (int i = 0; i < size - 1; ++i) {
		for (int j = i + 1; j < size; ++j) {
			if (GroupDistance((*taskGroups)[i], (*taskGroups)[j]) < range) {
				endflag = false;
				i = size - 1;
				break;
			}
		}
	}
	return endflag;
}

int RandomNoPair::GroupDistance(TaskSubgroup group1, TaskSubgroup group2) {
	// return the minimum distance between two robot grooups
	int mini_distance = INT_MAX;
	for (int i = 0; i < group1.ltaskNumber; ++i) {
		TaskPoint* task1 = group1.ltasks[i];
		for (int j = 0; j < group2.ltaskNumber; ++j) {
			TaskPoint* task2 = group2.ltasks[j];

			// calculate the distance between the robots in two groups
			int temp_dist = abs(task1->taskPoint.x - task2->taskPoint.x)
				+ abs(task1->taskPoint.y - task2->taskPoint.y);

			// get the minimum distance
			if (temp_dist < mini_distance) mini_distance = temp_dist;
		}
	}
	return mini_distance;
}
