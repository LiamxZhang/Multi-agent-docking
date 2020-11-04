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

class SAPOAads {
public:
	// main function
	void Processing(string data_dir);

	// supporting functions
	bool TaskExtension(Task* task, MatrixMap* map);
	int _RobotMove(Task* task, vector<Robot*> robot, MatrixMap* world);

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

void SAPOAads::Processing(string data_dir) {
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
	//AssignTaskToRobot(task, robot);
	HungarianAssign(task, robot,world);

	// show the task extension process
	RecordTaskExtend(task, robot);
	
	// Robot movement
	//RobotMove(task, robot, world, tID2index);
	robotStep_sys = _RobotMove(task, robot, world);
	if (!robotStep_sys) {
		isComplete = false;
		return;
	}
	
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

bool SAPOAads::TaskExtension(Task* task, MatrixMap* map) {
	task->PushAll("allExtendedPoints");
	vector<int> sepStuckGroups;
	vector<int> moveStuckGroups;
	taskStep_sys = 0;
	int depth = task->AssemblyTree.depth(task->AssemblyTree.root());
	for (int i = 0; i < depth - 1; i++) { // the leaf layer of assembly tree is not needed for extension
		// construct the task subgroups
		vector<TaskSubgroup>* taskGroups = new vector<TaskSubgroup>();
		GetTaskSubgroups(taskGroups, task->AssemblyTree.root(), task->SegTree.root(), task, 0, i);

		cout << endl << endl << "Depth proess:   " << i << endl
			<< "How many groups :  " << taskGroups->size() << endl;
		bool sepComplete = false;
		vector<int> collision;
		int step = 0;
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

			// Fail check
			taskStep_sys++;
			if (taskStep_sys > DEADLOOP) {
				Recover(task);
				cout << endl << endl << "Error: System failed!!!" << endl;
				return false;
			}
			CheckFail(task) ? step++ : step = 0;
			if (step > 10) {
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


// robot groups move, local path replanning
int SAPOAads::_RobotMove(Task* task, vector<Robot*> robot, MatrixMap* world) {
	// initialize robot groups
	vector<RobotGroup> groups;
	vector<int> robotIDs;
	for (int i = 0; i < robot.size(); i++) {  // for each one node 
		vector<Robot*> tempGroup;
		tempGroup.push_back(robot[i]);
		RobotGroup robotGroup(tempGroup);  // robot group
		groups.push_back(robotGroup);
		robotIDs.push_back(robot[i]->id);
	}

	// robots move
	int stepNum = task->allTargets.size(); // tree depth
	int roboNum = task->allTargets[0].size();
	int deadLoop = 0; // cout
	for (int i = 0; i < stepNum; i++) { // for each one layer
		// update task
		task->UpdateTaskmap(world, stepNum - i - 1);

		// initial path planning without robots in the map
		for (int j = 0; j < groups.size(); j++) {
			groups[j].LocalPathPlanning(world, task->allTargets[stepNum - i - 1], vector<int>(), vector<char>(), INT_MAX, 0);
		}

		// move
		int step = 0;
		bool reach = false;
		vector<int> state(groups.size());// flag indicates the robot group is 0: moving, 1: waiting or 2: replanning
		int waitRound = 0; // wait for how many round to replan the path

		while (!reach) {
			//
			for (int j = 0; j < groups.size(); j++) {
				//vector<int> peersIDs = GetPeers(groups[j], robot, task, stepNum - i);
				//vector<int> robotInPeerTarget = GetRobotAtPeerTarget(robot, world, task->allTargets[stepNum - i - 1], peersIDs);
				if (state[j] >= waitRound) {
					// task get the group target up,down,left,right
					// step: stepNum - i - 1, group: robot ids
					vector<char> segDir_childSide;
					
					groups[j].LocalPathPlanning(world, task->allTargets[stepNum - i - 1], robotIDs, vector<char>(), INT_MAX, 0);
					state[j] = 0;
				}

				waitRound = 1;
				// move
				groups[j].TrialMove();
				cout << "Trial move!" << endl;

				if (!world->CollisionCheck(groups[j].GetTendPos(), groups[j].GetRobotIds(), vector<int> (), vector<Point> (), 0)
					&& groups[j].robot[groups[j].leaderIndex]->planPath.size()) {
					groups[j].Move(world);
					cout << "Robots move!" << endl;
					world->Display("robot");
				}
				else { // wait for one more round
					cout << "Collision!" << endl;
					groups[j].leaderIndex + 1 > groups[j].robotNumber - 1 ?
						groups[j].AssignLeaders(0) : groups[j].AssignLeaders(groups[j].leaderIndex + 1);
					state[j] += 1;
				}
			}
			// check, if leader robots reach targets, reach = true
			reach = CheckReach(groups);
			RecordRobotPosition(robot);

			// check whether fail
			deadLoop++;
			if (deadLoop > DEADLOOP) {
				Recover(task);
				cout << endl << endl << "Error: System failed!!!" << endl;
				return 0;
			}
			CheckFail(robot) ? step++ : step = 0;
			if (step > 10) {
				return 0;
			}
		}

		world->Display("all");
		// dock
		groups = Dock(robot, task, stepNum - i - 1);
	}
	return deadLoop;
}

