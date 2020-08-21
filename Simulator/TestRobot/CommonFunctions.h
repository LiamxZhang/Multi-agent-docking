#pragma once
#include <iostream>
#include <vector>
#include <windows.h>

#include "Task.h"
#include "Robot.h"
#include "Hungarian.h"

#define random() (rand() / double(RAND_MAX))
#define WaitTime 0
#define DEADLOOP 3000
#define REPEAT 30
#define LogFlag false

using namespace std;

// About Task //
#pragma region Task


static void GetTaskSubgroups(vector<TaskSubgroup>* taskGroups, BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, int depth, int obj) {
	if (!assNode) return;   // tree node empty
	if (assNode->data.size() <= 1) return;  // cannot be extended anymore

	if (depth == obj) {
		// construct the taskgroups
		vector<int> lcomponents = assNode->lChild->data;
		vector<int> rcomponents = assNode->rChild->data;
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

	GetTaskSubgroups(taskGroups, assNode->lChild, segNode->lChild, task, depth + 1, obj);
	GetTaskSubgroups(taskGroups, assNode->rChild, segNode->rChild, task, depth + 1, obj);
}


#pragma endregion

// Assignment from task to robot // 
#pragma region Assignment

// assign the task to the closest robots using optimization (or bid)
// from task->allTargets[j][i]->taskpoint.x(y)
// to robot[i]->initPosition.x(y)
// return the assigned task IDs corresponding to the robot index
static vector<int> AssignTaskToRobot(Task* task, vector<Robot*> robot) {
	float closestDistance;
	float distance;   // distance between robots and tasks
	bool assigned;
	vector<int> assignedTaskID;
	for (int i = 0; i < task->robotNum; i++) {
		closestDistance = 100000;   // max is initialized as infinity
		int roboX = robot[i]->initPosition.x;
		int roboY = robot[i]->initPosition.y;
		for (int j = 0; j < task->taskNum; j++) {
			// if task has been assigned, break
			assigned = false;
			for (int k = 0; k < assignedTaskID.size(); k++)
				if (assignedTaskID[k] == task->allTargets.back()[j]->id) {
					assigned = true;
					break;
				}
			if (assigned) continue;
			// calculate the distance
			int taskX = task->allTargets.back()[j]->taskPoint.x;
			int taskY = task->allTargets.back()[j]->taskPoint.y;
			distance = abs(roboX - taskX) + abs(roboY - taskY); // Manhattan distance
			//
			//cout << endl << "X: " << roboX << "    Y: " << roboY << "   distance: " << distance;
			if (distance < closestDistance) {
				closestDistance = distance;
				robot[i]->taskID = task->allTargets.back()[j]->id;
			}
		}
		assignedTaskID.push_back(robot[i]->taskID);
	}
	// see the assignment
	cout << endl;
	for (int i = 0; i < task->robotNum; i++) {
		cout << "Assign robot " << robot[i]->id << ": task ID " << robot[i]->taskID << endl;
	}
	cout << endl;
	if (LogFlag)
		RecordLog("Success to assign tasks to robots!");

	return assignedTaskID;
}

static vector<int> HungarianAssign(Task* task, vector<Robot*> robot, MatrixMap* world) {
	// Construct the cost matrix
	vector< vector<double> > costMatrix;
	for (int i = 0; i < robot.size(); ++i) {
		vector<double> tempPathLength;
		// update map
		robot[i]->UpdateLocalMap(world, vector<int>(), vector<int>());
		for (int j = 0; j < task->allTargets.back().size(); ++j) {
			// update target position
			robot[i]->targetPosition = task->allTargets.back()[j]->taskPoint;

			// robot path planning
			vector<Point> planPath = robot[i]->AStarPath();
			tempPathLength.push_back(double(planPath.size()));
		}
		costMatrix.push_back(tempPathLength);
	}
	
	// Hungarian Solver
	HungarianAlgorithm HungAlgo;
	vector<int> assignment;

	double cost = HungAlgo.Solve(costMatrix, assignment);

	// Assignment
	vector<int> assignedTaskID;
	for (int i = 0; i < robot.size(); i++) {
		robot[i]->taskID = task->allTargets.back()[assignment[i]]->id;
		assignedTaskID.push_back(robot[i]->taskID);
	}
	return assignedTaskID;
}

#pragma endregion

// About Robot //
#pragma region Robot


// get the members' IDs of robot group
static vector<int> GetPeers(RobotGroup group, vector<Robot*> robot, Task* task, vector<int> tID2index, int layer) {
	vector<BinNode<vector<int>>*> nodeVec = task->AssemblyTree.getLayerNode(task->AssemblyTree.root(), 0, layer, nodeVec);
	vector<int> memberIDs;
	for (int i = 0; i < nodeVec.size(); i++) { // check each group
		bool isPeer = false;
		for (int j = 0; j < nodeVec[i]->data.size(); j++) { // each ID
			int robotID = robot[tID2index[nodeVec[i]->data[j]]]->id;
			memberIDs.push_back(robotID);
			if (group.robot[0]->id == robotID) isPeer = true; // pair the group
		}
		if (isPeer) return memberIDs;
		memberIDs.swap(vector<int>());
	}
	return memberIDs;
}

// get the peers' IDs of robot group
static vector<int> GetPeers(RobotGroup group, vector<Robot*> robot, Task* task, int layer) {
	// in case empty group
	if (!group.robot.size())
		return vector<int> ();

	// get the task peer IDs
	vector<int> taskIDs = task->GetPeers(task->AssemblyTree.root(), vector<int>(),
		group.robot[0]->taskID, 0, layer); // to be docked group

	// find back the robot IDs
	vector<int> peerIDs;
	for (int i = 0; i < robot.size(); i++) {
		vector<int>::iterator iter = find(taskIDs.begin(), taskIDs.end(), robot[i]->taskID);
		if (iter != taskIDs.end())
			peerIDs.push_back(robot[i]->id);
	}
	return peerIDs;
}

// dock // put robots into one group
static vector<RobotGroup> Dock(vector<Robot*> robot, Task* task, vector<int> tID2index, int layer) {  // layer = stepNum - i - 1
	// get nodes of one layer, task->AssemblyTree  // all nodes of one layer of task tree
	vector<BinNode<vector<int>>*> nodeVec = task->AssemblyTree.getLayerNode(task->AssemblyTree.root(), 0, layer, nodeVec);
	// see the nodes
	cout << "all groups in nodes: ";
	for (int j = 0; j < nodeVec.size(); j++) {
		cout << "(";
		for (int k = 0; k < nodeVec[j]->data.size(); k++) {
			cout << nodeVec[j]->data[k] << ",";
		}
		cout << "),";
	}
	cout << endl;

	// push robots into one group
	vector<RobotGroup> groups;
	for (int j = 0; j < nodeVec.size(); j++) {  // for each one node 
		vector<Robot*> tempGroup;
		for (int k = 0; k < nodeVec[j]->data.size(); k++) {  // for each one robot
			tempGroup.push_back(robot[tID2index[nodeVec[j]->data[k]]]); // int taskID = nodeVec[j]->data[k];
		}
		RobotGroup robotGroup(tempGroup);  // robot group
		groups.push_back(robotGroup);
	}
	// see the groups 
	for (int j = 0; j < groups.size(); j++) {
		//cout << "group " << j << ":";  groups[j].Display();  cout << endl;
		groups[j].AssignLeaders(0); // assign the leaders as the first robot 
	}
	if (LogFlag)
		RecordLog("Finished to dock!");
	cout << endl << "Finished to dock!" << endl;
	return groups;
}

static vector<RobotGroup> Dock(vector<Robot*> robot, Task* task, int layer) {  // layer = stepNum - i - 1
	// get nodes of one layer, task->AssemblyTree  // all nodes of one layer of task tree
	vector<BinNode<vector<int>>*> nodeVec = task->AssemblyTree.getLayerNode(task->AssemblyTree.root(), 0, layer, nodeVec);
	// see the nodes
	cout << "all task groups in nodes: ";
	for (int j = 0; j < nodeVec.size(); j++) {
		cout << "(";
		for (int k = 0; k < nodeVec[j]->data.size(); k++) {
			cout << nodeVec[j]->data[k] << ",";
		}
		cout << "),";
	}
	cout << endl;

	// push robots into one group
	vector<RobotGroup> groups;
	for (int j = 0; j < nodeVec.size(); j++) {  // for each one node 
		vector<Robot*> tempGroup;
		for (int k = 0; k < nodeVec[j]->data.size(); k++) {  // for each one task
			// which robot for the task of nodeVec[j]->data[k]
			for (int i = 0; i < robot.size(); i++) {
				if (robot[i]->taskID == nodeVec[j]->data[k])
					tempGroup.push_back(robot[i]); // int taskID = nodeVec[j]->data[k];
			}
		}
		RobotGroup robotGroup(tempGroup);  // robot group
		groups.push_back(robotGroup);
	}
	// see the groups 
	for (int j = 0; j < groups.size(); j++) {
		//cout << "group " << j << ":";  groups[j].Display();  cout << endl;
		groups[j].AssignLeaders(0); // assign the leaders as the first robot 
	}
	if (LogFlag)
		RecordLog("Finished to dock!");
	cout << endl << "Finished to dock!" << endl;
	return groups;
}


// task ID->robot index // robot ID -> robot index
static vector<vector<int>> IDtoIndex(vector<Robot*> robot) {
	int roboNum = robot[0]->robotNumber;  // task number = robot number
	vector<int> tID2index; // task ID->robot index
	vector<int> rID2index; // robot ID -> robot index
	vector<vector<int>> ID2index;  // ID2index[0] = tID2index  // ID2index[1] = rID2index; 
	for (int i = 0; i <= roboNum; i++) {
		{
			int taskID = i;   // task ID : 1~task number
			int index = 0;
			for (int j = 0; j < roboNum; j++)
				if (robot[j]->taskID == taskID) { index = j;  break; }
			tID2index.push_back(index);
			//cout << "task ID: " << taskID << "   index: " << index << endl;
		}
		{
			int robotID = i;   // robot ID : 1~robot number
			int index = 0;
			for (int j = 0; j < roboNum; j++)
				if (robot[j]->id == robotID) { index = j;  break; }
			rID2index.push_back(index);
			//cout << "robot ID: " << robotID << "   index: " << ind << endl;
		}
	}
	ID2index.push_back(tID2index);
	ID2index.push_back(rID2index);
	return ID2index;
}

// check all the leader robots whether they reach their targets
static bool CheckReachForLeader(vector<RobotGroup> groups) {
	for (int i = 0; i < groups.size(); i++) {
		if (groups[i].robot[0]->currentPosition.x != groups[i].robot[0]->targetPosition.x
			|| groups[i].robot[0]->currentPosition.y != groups[i].robot[0]->targetPosition.y) {
			return false;
		}
	}
	return true;
}

// check all the robot points whether they reach their targets
static bool CheckReach(vector<RobotGroup> groups) {
	for (int i = 0; i < groups.size(); ++i) {
		for (int j = 0; j < groups[i].robotNumber; ++j) {
			if (groups[i].robot[j]->currentPosition.x != groups[i].robot[j]->targetPosition.x
				|| groups[i].robot[j]->currentPosition.y != groups[i].robot[j]->targetPosition.y) {
				return false;
			}
		}
	}
	return true;
}

// check whether fail
static int CheckFail(vector<Robot*> robot) {
	int result = 1; // assume equal
	for (int i = 0; i < robot.size(); ++i) {
		if (robot[i]->lastPosition.x != robot[i]->currentPosition.x
			|| robot[i]->lastPosition.y != robot[i]->currentPosition.y) {
			result = 0; // not euqal
		}
	}
	return result;
}
static int CheckFail(Task* task) {
	int result = 1; // equal
	for (int i = 0; i < task->taskNum; ++i) {
		if (task->currentTargets[i]->taskPoint.x != task->currentTargets[i]->lastPosition.x
			|| task->currentTargets[i]->taskPoint.y != task->currentTargets[i]->lastPosition.y) {
			result = 0; // not euqal
		}
	}
	return result;
}

// check whether the robot group has accessed the target area
static bool CheckAccessTarget(RobotGroup robotGroup, vector<TaskPoint*> allTargets) {
	// allTargets: all targets in this round
	// find the current positions of robots
	vector<Point> currentPositions;
	for (int i = 0; i < robotGroup.robot.size(); i++) {
		currentPositions.push_back(robotGroup.robot[i]->currentPosition);
	}

	// find the target area
	vector<Point> targetPositions;
	for (int i = 0; i < robotGroup.robot.size(); i++)
		for (int j = 0; j < allTargets.size(); j++)
			if (robotGroup.robot[i]->taskID == allTargets[j]->id) {
				targetPositions.push_back(allTargets[j]->taskPoint);
			}

	// calculate the distance
	int distance = INT_MAX;
	for (int i = 0; i < currentPositions.size(); i++)
		for (int j = 0; j < targetPositions.size(); j++) {
			int temp = abs(currentPositions[i].x - targetPositions[j].x) + abs(currentPositions[i].y - targetPositions[j].y);
			if (temp < distance)
				distance = temp;
		}

	// return
	if (distance)
		return false;
	else if (!distance)
		return true;
}

// check whether all the robots have accessed their target points, only one unit distance away
static bool PeerTargetLock(RobotGroup robotGroup) {
	// return true: open lock; false: close lock
	Point currentPosition = robotGroup.robot[robotGroup.leaderIndex]->currentPosition;
	Point targetPosition = robotGroup.robot[robotGroup.leaderIndex]->targetPosition;
	// calculate the distance
	int distance = abs(currentPosition.x - targetPosition.x) + abs(currentPosition.y - targetPosition.y);
	if (distance <= 1)
		return true;
	else
		return false;
}

#pragma endregion

// Recording robot movement //
#pragma region Record

// for real-time, record the extended position of task points
static void RecordTaskExtendRT(Task* task, vector<Robot*> robots) {
	ofstream f;
	f.open("../TestRobot/Robot_Current_Position.txt", ofstream::out);
	if (f) {
		f << robots.size() << endl;
		//RecordLog("RecordCurrentAndTargetPosition:");
		for (int i = 0; i < robots.size(); ++i) {
			f << robots[i]->id << "," << robots[i]->currentPosition.x + 1 << "," << robots[i]->currentPosition.y + 1 << ","
				<< task->currentTargets[i]->taskPoint.x + 1 << "," << task->currentTargets[i]->taskPoint.y + 1 << endl;
		}
	}
	f.close();
	Sleep(WaitTime);
}

// record the extended position of task points, offline from the robots' allTargets
static void RecordTaskExtend(Task* task, vector<Robot*> robots) {
	vector<int> tIDs;
	for (int i = 0; i < robots.size(); ++i)
		for (int j = 0; j < task->allExtendedPoints[0].size(); ++j)
			if (task->allExtendedPoints[0][j]->id == robots[i]->taskID) {
				tIDs.push_back(j);
				break;
			}
	// tIDs依次保存robot[0]-[end]的对应的allTargets的ID
	for (int time = 0; time < task->allExtendedPoints.size(); ++time) {
		ofstream f;
		f.open("../TestRobot/Robot_Current_Position.txt", ofstream::out);
		if (f) {
			f << robots.size() << endl;
			//RecordLog("RecordCurrentAndTargetPosition:");
			for (int i = 0; i < robots.size(); ++i) {
				f << robots[i]->id << "," << robots[i]->currentPosition.x + 1 << ","
					<< robots[i]->currentPosition.y + 1 << ","
					<< task->allExtendedPoints[time][tIDs[i]]->taskPoint.x + 1 << ","
					<< task->allExtendedPoints[time][tIDs[i]]->taskPoint.y + 1 << endl;
			}
		}
		f.close();
		Sleep(WaitTime);
	}
}


// record the current position of robots
static bool RecordRobotPosition(vector<Robot*> robots) {
	ofstream f;
	f.open("../TestRobot/Robot_Current_Position.txt", ofstream::out);
	if (f) {
		f << robots.size() << endl;
		//RecordLog("RecordCurrentAndTargetPosition:");
		//RecordLog("RobotId   CurrentPosition   TargetPosition");
		for (int i = 0; i < robots.size(); ++i) {
			f << robots[i]->id << "," << robots[i]->currentPosition.x + 1 << "," << robots[i]->currentPosition.y + 1 << ","
				<< robots[i]->targetPosition.x + 1 << "," << robots[i]->targetPosition.y + 1 << endl;
			//RecordLog("   " + to_string(i) + "          [" + to_string(robotCurrentPosition[i - 1].x + 1) + ","
			//	+ to_string(robotCurrentPosition[i - 1].y + 1) + "]              [" + to_string(robotTargetPosition[i - 1].x + 1) +
			//	"," + to_string(robotTargetPosition[i - 1].y + 1) + "]");
		}
	}
	else
		return false;
	f.close();
	Sleep(WaitTime);
	return true;
}

// recover the task and robot data
static void Recover(Task* task) {
	ofstream f;
	f.open("../TestRobot/Robot_Current_Position.txt", ofstream::out);
	if (f) {
		f << task->robotNum << endl;
		for (int i = 0; i < task->robotNum; ++i) {
			f << task->startPoints[i]->id << "," << task->startPoints[i]->taskPoint.x + 1 << ","
				<< task->startPoints[i]->taskPoint.y + 1 << "," << task->finalTargets[i]->taskPoint.x + 1
				<< "," << task->finalTargets[i]->taskPoint.y + 1 << endl;
		}
	}
	f.close();
	
	/*
	f.open("../TestRobot/Task.txt", ofstream::out);
	if (f) {
		f << task->taskNum << endl;
		for (int i = 0; i < task->taskNum; i++) {
			//
			f << task->finalTargets[i]->id << " " << task->finalTargets[i]->taskPoint.x << " "
				<< task->finalTargets[i]->taskPoint.y << endl;
			// Log
		}
	}
	f.close();
	*/
}

// record the step
static vector<int> RecordStep(Task* task, vector<Robot*> robot) {
	vector<int> step;
	int max = INT_MIN;
	for (int i = 0; i < task->taskNum; ++i) {
		if (task->currentTargets[i]->step > max) {
			max = task->currentTargets[i]->step;
		}
	}
	step.push_back(max);
	max = INT_MIN;
	for (int i = 0; i < robot.size(); ++i) {
		if (robot[i]->step > max)
			max = robot[i]->step;
	}
	step.push_back(max);
	return step;
}

#pragma endregion


//
#pragma region Ajacent
// return the least distance between groups
static int GroupDistance(RobotGroup group1, RobotGroup group2) {
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

// check whether groups adjacent with each other
static bool CheckAdjacent(vector<RobotGroup> groups) {
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

// find all adjacent groups in a recursion way
static vector<int> AdjacentGroups(vector<RobotGroup> groups, vector<int> currentIDs, vector<int> allIDs) {
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
static vector<RobotGroup> FormGroup(vector<vector<int>> groupIDs, vector<RobotGroup> groups) { // group IDs
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

// return the robot IDs to be join in the same group
static vector<RobotGroup> NewGroups(vector<RobotGroup> groups, vector<Robot*> robot) {
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

#pragma endregion


static vector<int> GetRobotAtPeerTarget(vector<Robot*> robot, MatrixMap* world, vector<TaskPoint*> allTargets, vector<int> peersIDs) {
	// get peer robots
	vector<Robot*> peers;
	for (int k = 0; k < robot.size(); ++k) {
		vector<int>::iterator iter = find(peersIDs.begin(), peersIDs.end(), robot[k]->id);
		if (iter != peersIDs.end()) peers.push_back(robot[k]);
	}
	// get peer target area
	vector<Point> peerTarget;
	for (int i = 0; i < peers.size(); i++)
		for (int j = 0; j < allTargets.size(); j++)
			if (peers[i]->taskID == allTargets[j]->id) {
				peerTarget.push_back(allTargets[j]->taskPoint);
				break;
			}
	// find the robots in peer target area
	vector<int> robotInPeerTarget;
	for (int i = 0; i < peerTarget.size(); i++) {
		if (world->map_robot(peerTarget[i].x, peerTarget[i].y))
			robotInPeerTarget.push_back(world->map_robot(peerTarget[i].x, peerTarget[i].y));
	}

	return robotInPeerTarget;
}

// robots move
static void RobotMove(Task* task, vector<Robot*> robot, MatrixMap* world, vector<int> tID2index) {
	// initialize robot groups
	vector<RobotGroup> groups;
	for (int i = 0; i < robot.size(); i++) {  // for each one node 
		vector<Robot*> tempGroup;
		tempGroup.push_back(robot[i]);
		RobotGroup robotGroup(tempGroup);  // robot group
		groups.push_back(robotGroup);
	}

	// robots move
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
			for (int j = 0; j < groups.size(); j++) {
				vector<int> peersIDs = GetPeers(groups[j], robot, task, tID2index, stepNum - i - 1); // same group and to be docked group
				groups[j].PathPlanning(world, task->allTargets[stepNum - i - 1], peersIDs);
				groups[j].TrialMove();
				if (!world->CollisionCheck(groups[j].GetTendPos(), groups[j].GetRobotIds(), peersIDs)) {
					groups[j].Move(world);
					world->Display("robot");
				}
			}

			// check, if leader robots reach targets, reach = true
			reach = CheckReach(groups);
			RecordRobotPosition(robot);
		}
		world->Display("all");
		string str1 = "Finished to move all robots to the targets of layer ";
		string str2 = to_string(stepNum - i - 1);
		if (LogFlag)
			RecordLog(str1 + str2);
		cout << str1 + str2 << endl;
		//system("pause");
		// dock
		groups = Dock(robot, task, tID2index, stepNum - i - 1);
	}
}


// robot groups move, local path replanning
static bool RobotMove_LocalPlan(Task* task, vector<Robot*> robot, MatrixMap* world) {
	// initialize robot groups
	vector<RobotGroup> groups;
	for (int i = 0; i < robot.size(); i++) {  // for each one node 
		vector<Robot*> tempGroup;
		tempGroup.push_back(robot[i]);
		RobotGroup robotGroup(tempGroup);  // robot group
		groups.push_back(robotGroup);
	}

	// robots move
	int stepNum = task->allTargets.size(); // tree depth
	int roboNum = task->allTargets[0].size();
	for (int i = 0; i < stepNum; i++) { // for each one layer
		// update task
		task->UpdateTaskmap(world, stepNum - i - 1);
		
		// initial path planning without robots in the map
		for (int j = 0; j < groups.size(); j++) {
			groups[j].LocalPathPlanning(world, task->allTargets[stepNum - i - 1], vector<int> (), vector<char> ());
		}

		// move
		int step = 0;
		bool reach = false;
		vector<int> state(groups.size());// flag indicates the robot group is 0: moving, 1: waiting or 2: replanning
		int waitRound = 0; // wait for how many round to replan the path
		int deadLoop = 0;
		while (!reach) {
			//
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
			//
			for (int j = 0; j < groups.size(); j++) {
				vector<int> peersIDs = GetPeers(groups[j], robot, task, stepNum - i);
				vector<int> robotInPeerTarget = GetRobotAtPeerTarget(robot, world, task->allTargets[stepNum - i - 1], peersIDs);
				if (state[j] >= waitRound) {
					// task get the group target up,down,left,right
					// step: stepNum - i - 1, group: robot ids
					vector<char> segDir_childSide;
					/*
					char segdir = task->GetSegDirection(task->SegTree.root(), task->AssemblyTree.root(), char(), groups[j].robot[0]->taskID, 0, stepNum - i);
					char childside = task->GetChildSide(task->AssemblyTree.root(), char(), groups[j].robot[0]->taskID, 0, stepNum - i);
					
					segDir_childSide.push_back(segdir);
					segDir_childSide.push_back(childside);
					cout << endl << "Seg direction:\t" << segdir << "\tChild side:\t" << childside << endl;
					*/
					if (PeerTargetLock(groups[j])) { // open for peer target area
						groups[j].LocalPathPlanning(world, task->allTargets[stepNum - i - 1], robotInPeerTarget, segDir_childSide);
					}
					else {
						groups[j].LocalPathPlanning(world, task->allTargets[stepNum - i - 1], vector<int>(), segDir_childSide);
					}
					state[j] = 0;
				}

				waitRound = 1;
				// move
				groups[j].TrialMove();
				cout << "Trial move!" << endl;
				vector<int> openIDs; // robots at the peer target area
				PeerTargetLock(groups[j]) ? openIDs = robotInPeerTarget : openIDs = vector<int>();

				if (!world->CollisionCheck(groups[j].GetTendPos(), groups[j].GetRobotIds(), openIDs, groups[j].addObstacles)
					&& groups[j].robot[groups[j].leaderIndex]->planPath.size()) {
					groups[j].Move(world);
					world->Display("robot");
				}
				else { // wait for one more round
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
				return false;
			}
			CheckFail(robot) ? step++ : step = 0;
			if (step > 10) {
				return false;
			}
		}

		world->Display("all");
		// dock
		groups = Dock(robot, task, stepNum - i - 1);
	}
	return true;
}

