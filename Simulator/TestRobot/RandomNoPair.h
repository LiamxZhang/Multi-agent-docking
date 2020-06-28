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

class RandomNoPair {
public:
	// main function
	void Processing();

	// supporting functions
	void TaskExtension(Task* task, MatrixMap* map);
	void PrepareTaskSubgroups(vector<TaskSubgroup>* taskGroups, BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, int depth, int obj);
	bool EndCheck(vector<TaskSubgroup>* taskGroups, int range);
	int GroupDistance(TaskSubgroup group1, TaskSubgroup group2);
	
private:
};


void RandomNoPair::Processing() {
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
	RobotMove(task, robot, world, tID2index);
	
	//system("pause");
	Recover(task);
	return;
}

void RandomNoPair::TaskExtension(Task* task, MatrixMap* map) {
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
		}

		// display
		cout << endl << "Extend step " << i << " : " << endl;
		map->Display("task");
		cout << endl;
		task->PushAll("allTargets");
	}
}

// prepare the task groups
void RandomNoPair::PrepareTaskSubgroups(vector<TaskSubgroup>* taskGroups, BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, int depth, int obj) {
	if (!assNode) return;   // tree node empty

	if (depth == obj) {
		// construct the taskgroups
		//vector<int> lcomponents = assNode->lChild->data;
		//vector<int> rcomponents = assNode->rChild->data;

		vector<int> lcomponents = assNode->data;
		vector<int> rcomponents;

		vector<TaskPoint*> ltask;
		vector<TaskPoint*> rtask;
		bool belongToLeft;
		bool lfull = false;
		bool rfull = false;
		for (int i = 0; i < task->currentTargets.size(); ++i) {
			// lcomponents
			belongToLeft = false;
			if (ltask.size() < lcomponents.size()) {
				for (int j = 0; j < lcomponents.size(); ++j)
					if (task->currentTargets[i]->id == lcomponents[j]) {
						ltask.push_back(task->currentTargets[i]);
						belongToLeft = true;
						break;
					}
			}
			else
				lfull = true;
			if (belongToLeft) continue;

			// rcomponents
			if (rtask.size() < rcomponents.size()) {
				for (int j = 0; j < rcomponents.size(); ++j)
					if (task->currentTargets[i]->id == rcomponents[j]) {
						rtask.push_back(task->currentTargets[i]);
						break;
					}
			}
			else
				rfull = true;
			if (lfull && rfull) break;
		}
		// initialization
		TaskSubgroup taskgroup(ltask, rtask, segNode->data, 3);  // range = 2
		taskGroups->push_back(taskgroup);
	}

	if (assNode->data.size() <= 1) return;  // cannot be extended anymore

	GetTaskSubgroups(taskGroups, assNode->lChild, segNode->lChild, task, depth + 1, obj);
	GetTaskSubgroups(taskGroups, assNode->rChild, segNode->rChild, task, depth + 1, obj);
}

bool RandomNoPair::EndCheck(vector<TaskSubgroup>* taskGroups, int range) {
	bool endflag = true;
	for (int i = 0; i < taskGroups->size() - 1; ++i) {
		for (int j = i + 1; j < taskGroups->size(); ++j) {
			if (GroupDistance((*taskGroups)[i], (*taskGroups)[j]) < range) {
				endflag = false;
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
