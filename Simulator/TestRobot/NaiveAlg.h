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

class NaiveAlg {
public:
	// main function
	void Processing();

	// supporting functions
	void NaiveRobotMove(Task* task, vector<Robot*> robot, MatrixMap* world);

	vector<RobotGroup> NewGroups(vector<RobotGroup> groups, vector<Robot*> robot);
	vector<int> AdjacentGroups(vector<RobotGroup> groups, vector<int> currentIDs, vector<int> allIDs);
	vector<RobotGroup> FormGroup(vector<vector<int>> groupIDs, vector<RobotGroup> groups);
	bool CheckAdjacent(vector<RobotGroup> groups);
	int GroupDistance(RobotGroup group1, RobotGroup group2);

private:
};


void NaiveAlg::Processing() {
	// read map, the origin is in the leftmost top,  x means rows, y means columns
	MatrixMap* world = new MatrixMap();
	world->ReadMap();
	world->Display("obstacle"); //world.Display();

	// read task, generate assembly tree
	Task* task = new Task();
	task->ReadTask();
	//task->GenerateTree();
	//cout << endl << "Depth: " << task->AssemblyTree.depth(task->AssemblyTree.root()) << endl;
	//cout << endl << world->TaskCheck(1, task->AssemblyTree.leaves()[0]->data, 2) << endl; 

	// create the robots
	vector<Robot*> robot;
	for (int i = 0; i < task->robotNum; i++) {
		Robot* temp = new Robot(task->robotNum, task->startPoints[i]->id, task->startPoints[i]->taskPoint, world->RowNum, world->ColNum);
		robot.push_back(temp);
	}

	// Extend the task components, according to the assembly tree
	//TaskExtension(task, world);

	// assign the task to the closest robots using optimization (or bid)
	//AssignTaskToRobot(task, robot);
	HungarianAssign(task, robot, world);
	// show the task extension process
	//RecordTaskExtend(task, robot); 

	NaiveRobotMove(task, robot, world);

	//system("pause");
	Recover(task);
	return;
}

void NaiveAlg::NaiveRobotMove(Task* task, vector<Robot*> robot, MatrixMap* world) {
	// initialize robot groups
	vector<RobotGroup> groups;
	for (int i = 0; i < robot.size(); i++) {  // for each one node 
		vector<Robot*> tempGroup;
		tempGroup.push_back(robot[i]);
		RobotGroup robotGroup(tempGroup);  // robot group
		groups.push_back(robotGroup);
	}

	// robots move
	int step = 0; // count the steps
	int stepNum = task->allTargets.size(); // tree depth
	int roboNum = task->allTargets[0].size();
	for (int i = 0; i < stepNum; i++) { // for each one layer
		cout << "In step " << i + 1 << " of move:" << endl;
		task->Display(stepNum - i - 1);
		// update task
		task->UpdateTaskmap(world, stepNum - i - 1);
		// move
		bool reach = false;
		while (!reach) {
			// check whether adjacent // if adjacent, form new groups
			if (CheckAdjacent(groups)) {
				vector<RobotGroup> newgroups = NewGroups(groups, robot);
				groups = newgroups;
				cout << "Form the new groups! " << endl;
				cout << "New Group IDs :" << endl;
				for (int i = 0; i < groups.size(); ++i) {
					cout << "group " << i << " :\t";
					for (int j = 0; j < groups[i].robotNumber; ++j) {
						cout << groups[i].robot[j]->id << ",\t";
					}
					cout << endl;
				}
				cout << endl;
			}

			// group move
			for (int j = 0; j < groups.size(); j++) {
				//vector<int> peersIDs = GetPeers(groups[j], robot, task, tID2index, stepNum - i - 1); // same group and to be docked group
				vector<int> peersIDs;
				cout << "1" << endl;
				groups[j].PathPlanning(world, task->allTargets[stepNum - i - 1], peersIDs, 0, 0);
				cout << "2" << endl;
				groups[j].TrialMove();
				cout << "3" << endl;
				if (!world->CollisionCheck(groups[j].GetRobotPos(), groups[j].GetRobotIds(), peersIDs, 0)) { // free
					groups[j].Move(world);
					cout << "4" << endl;
					world->Display("robot");
				}
				else {
					int lID;
					groups[j].leaderIndex + 1 > groups[j].robotNumber - 1 ? lID = 0 : lID = groups[j].leaderIndex + 1;
					groups[j].AssignLeaders(lID);
				}
			}

			// check, if leader robots reach targets, reach = true
			reach = CheckReach(groups);
			RecordRobotPosition(robot);

			// check dead loop
			step += 1;
			if (step > 100) {
				Recover(task);
				cout << endl << endl << "Error: System failed!!!" << endl;
				return;
			}
		}
		world->Display("all");
		string str1 = "Finished to move all robots to the targets of layer ";
		string str2 = to_string(stepNum - i - 1);
		RecordLog(str1 + str2);
		cout << str1 + str2 << endl;
		//system("pause");
	}
}

// return the robot IDs to be join in the same group
vector<RobotGroup> NaiveAlg::NewGroups(vector<RobotGroup> groups, vector<Robot*> robot) {
	vector<vector<int>> groupIDs;

	vector<int> allIDs; // already counted groups
	for (int i = 0; i < groups.size(); ++i) {
		vector<int>::iterator result = find(allIDs.begin(), allIDs.end(), i);
		if (result != allIDs.end()) continue;

		vector<int> tempIDs;
		tempIDs.push_back(i);
		vector<int> new_IDs = AdjacentGroups(groups, tempIDs, tempIDs);

		groupIDs.push_back(new_IDs);
		for (int j = 0; j < new_IDs.size(); ++j) allIDs.push_back(new_IDs[j]);
	}
	/*
	cout << "New Group IDs :" << endl;
	for (int i = 0; i < groupIDs.size(); ++i) {
		cout << "group " << i << " :\t";
		for (int j = 0; j < groupIDs[i].size(); ++j) {
			for (int k = 0; k < groups[groupIDs[i][j]].robotNumber; ++k)
			cout << groups[groupIDs[i][j]].robot[k]->id << ",\t";
		}
		cout << endl;
	}
	cout << endl;
	*/
	return FormGroup(groupIDs, groups);
}

// find all adjacent groups in a recursion way
vector<int> NaiveAlg::AdjacentGroups(vector<RobotGroup> groups, vector<int> currentIDs, vector<int> allIDs) {
	// currentIDs, new added adjacent groups
	// allIDs, all the connected groups

	// find all adjacent groups
	vector<int> newIDs;
	for (int i = 0; i < currentIDs.size(); ++i) {
		int ID = currentIDs[i];
		for (int j = 0; j < groups.size(); ++j) {
			vector<int>::iterator result = find(allIDs.begin(), allIDs.end(), j);
			if (result != allIDs.end()) continue;

			if (GroupDistance(groups[j], groups[ID]) == 1) {
				newIDs.push_back(j);
				allIDs.push_back(j);
			}
		}
	}

	// end condition
	if (!newIDs.size()) return allIDs;

	// recursion
	vector<int> new_allIDs = AdjacentGroups(groups, newIDs, allIDs);

	return new_allIDs;
}


// for close robots, form a large group
vector<RobotGroup> NaiveAlg::FormGroup(vector<vector<int>> groupIDs, vector<RobotGroup> groups) { // group IDs
	vector<RobotGroup> new_group;
	for (int i = 0; i < groupIDs.size(); ++i) { // new group
		vector<Robot*> tempGroup;
		for (int j = 0; j < groupIDs[i].size(); ++j) { // old group
			for (int k = 0; k < groups[groupIDs[i][j]].robotNumber; ++k) { // robots in one old group
				tempGroup.push_back(groups[groupIDs[i][j]].robot[k]);
			}
		}
		RobotGroup robotGroup(tempGroup);  // robot group
		new_group.push_back(robotGroup);
	}

	return new_group;
}

// check whether groups adjacent with each other
bool NaiveAlg::CheckAdjacent(vector<RobotGroup> groups) {
	//
	for (int i = 0; i < groups.size() - 1; ++i) {
		for (int j = i + 1; j < groups.size(); ++j) {
			if (GroupDistance(groups[i], groups[j]) == 1) {
				return true;
			}
		}
	}
	return false;
}

// return the least distance between groups
int NaiveAlg::GroupDistance(RobotGroup group1, RobotGroup group2) {
	// return the minimum distance between two robot grooups
	int mini_distance = INT_MAX;
	for (int i = 0; i < group1.robotNumber; ++i) {
		Robot* robot1 = group1.robot[i];
		for (int j = 0; j < group2.robotNumber; ++j) {
			// calculate the distance between the robots in two groups
			Robot* robot2 = group2.robot[j];
			int temp_dist = abs(robot1->currentPosition.x - robot2->currentPosition.x)
				+ abs(robot1->currentPosition.y - robot2->currentPosition.y);

			// get the minimum distance
			if (temp_dist < mini_distance) mini_distance = temp_dist;
		}
	}
	return mini_distance;
}

