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

class NaiveAlg {
public:
	// main function
	void Processing(string data_dir);

	// supporting functions
	bool NaiveRobotMove(Task* task, vector<Robot*> robot, MatrixMap* world);

	//vector<RobotGroup> NewGroups(vector<RobotGroup> groups, vector<Robot*> robot);
	//vector<int> AdjacentGroups(vector<RobotGroup> groups, vector<int> currentIDs, vector<int> allIDs);
	//vector<RobotGroup> FormGroup(vector<vector<int>> groupIDs, vector<RobotGroup> groups);
	//bool CheckAdjacent(vector<RobotGroup> groups);
	//int GroupDistance(RobotGroup group1, RobotGroup group2);

	// variables
	double taskStep = 0;
	double robotStep = 0;
	double taskStep_mean = 0;
	double robotStep_mean = 0;
	double taskStep_variance = 0;
	double robotStep_variance = 0;
	double taskStep_sys = 0;
	double robotStep_sys = 0;
	bool isComplete = true;
private:
};


void NaiveAlg::Processing(string data_dir) {
	// read map, the origin is in the leftmost top,  x means rows, y means columns
	MatrixMap* world = new MatrixMap();
	world->ReadMap(data_dir);
	world->Display("obstacle"); //world.Display();

	// read task, generate assembly tree
	Task* task = new Task();
	task->ReadTask(data_dir);
	//task->GenerateTree();
	//cout << endl << "Depth: " << task->AssemblyTree.depth(task->AssemblyTree.root()) << endl;
	//cout << endl << world->TaskCheck(1, task->AssemblyTree.leaves()[0]->data, 2) << endl; 

	// create the robots
	vector<Robot*> robot;
	for (int i = 0; i < task->robotNum; i++) {
		Robot* temp = new Robot(task->robotNum, task->startPoints[i]->id, task->startPoints[i]->taskPoint, world->RowNum, world->ColNum);
		robot.push_back(temp);
	}

	// assign the task to the closest robots using optimization (or bid)
	//AssignTaskToRobot(task, robot);
	HungarianAssign(task, robot, world);

	isComplete = NaiveRobotMove(task, robot, world);
	if (!isComplete) return;

	//system("pause");
	Recover(task);

	//record the step
	vector<double> steps = RecordStep(task, robot);
	taskStep = steps[0];
	taskStep_mean = steps[1];
	taskStep_variance = steps[2];
	robotStep = steps[3];
	robotStep_mean = steps[4];
	robotStep_variance = steps[5];
}

bool NaiveAlg::NaiveRobotMove(Task* task, vector<Robot*> robot, MatrixMap* world) {
	// initialize robot groups
	vector<RobotGroup> groups;
	for (int i = 0; i < robot.size(); i++) {  // for each one node 
		vector<Robot*> tempGroup;
		tempGroup.push_back(robot[i]);
		RobotGroup robotGroup(tempGroup);  // robot group
		groups.push_back(robotGroup);
	}

	// robots move
	robotStep_sys = 0;
	int stepNum = task->allTargets.size(); // tree depth
	int roboNum = task->allTargets[0].size();
	vector<int> peersIDs;
	for (int i = 0; i < robot.size(); i++) peersIDs.push_back(robot[i]->id); // open for all group
	for (int i = 0; i < stepNum; i++) { // for each one layer
		cout << "In step " << i + 1 << " of move:" << endl;
		task->Display(stepNum - i - 1);
		// update task
		task->UpdateTaskmap(world, stepNum - i - 1);

		// move
		int step = 0; // count the steps
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
				groups[j].LocalPathPlanning(world, task->allTargets[stepNum - i - 1], peersIDs);
				groups[j].TrialMove();
				if (!world->CollisionCheck(groups[j].GetTendPos(), groups[j].GetRobotIds(), peersIDs, vector<Point>(), 0)) { // free
					groups[j].Move(world);
					world->Display("robot");
				}
				else {
					groups[j].leaderIndex + 1 > groups[j].robotNumber - 1 ? 
						groups[j].AssignLeaders(0) : groups[j].AssignLeaders(groups[j].leaderIndex + 1);
				}
			}

			// check, if leader robots reach targets, reach = true
			reach = CheckReach(groups);
			RecordRobotPosition(robot);

			// check dead loop
		
			robotStep_sys++;
			if (robotStep_sys > DEADLOOP) {
				Recover(task);
				cout << endl << endl << "Fail: System is in endless loop!!!" << endl;
				return false;
			}
			
			CheckFail(robot) ? step++ : step = 0;
			if (step > 10) {
				cout << endl << endl << "Fail: System cannot move!!!" << endl;
				return false;
			}

		}
		world->Display("all");
		string str1 = "Finished to move all robots to the targets of layer ";
		string str2 = to_string(stepNum - i - 1);
		if (LogFlag)
			RecordLog(str1 + str2);
		cout << str1 + str2 << endl;
		//system("pause");
	}
	return true;
}

