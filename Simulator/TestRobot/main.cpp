#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <ctime>
//#include <string>
//#include <thread>
#include <math.h>
#include <algorithm>
#include <queue>
#include <map>
#include <set>
//#include <mutex>
#include <windows.h>

#include "Point.h"
#include "Task.h"
#include "Map.h"
#include "Robot.h"
#include "Log.h"

#define random() (rand() / double(RAND_MAX))
#define WaitTime 500

using namespace std;

//void ExtendTask(BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, MatrixMap* map, int depth, int obj);
//void ExtendAction(BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, MatrixMap* map, int curDepth);
void TaskExtension(Task* task, MatrixMap* map);
void AssignWeights(vector<TaskSubgroup>* taskGroups, vector<int> sepStuckGroups);
void GetTaskSubgroups(vector<TaskSubgroup>* taskGroups, BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, int depth, int obj);
vector<int> AssignTaskToRobot(Task* task, vector<Robot*> robot);
vector<vector<int>> IDtoIndex(vector<Robot*> robot);
bool CheckReach(vector<RobotGroup> groups);
vector<RobotGroup> Dock(vector<Robot*> robot, Task* task, vector<int> tID2index, int layer);
vector<int> GetPeers(RobotGroup group, vector<Robot*> robot, Task* task, vector<int> tID2index, int layer);
// 
void RecordTaskExtendRT(Task* task, vector<Robot*> robots);
void RecordTaskExtend(Task* task, vector<Robot*> robots);
bool RecordRobotPosition(vector<Robot*> robots);
void Recover(Task* task);

int main() {
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
		cout << "In step " << i+1 << " of move:"<< endl;
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
				if (!world->CollisionCheck(groups[j].GetRobotPos(), groups[j].GetRobotIds(), peersIDs)) {
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
		RecordLog(str1 + str2);
		cout << str1 + str2 << endl;
		//system("pause");
		// dock
		groups = Dock(robot, task, tID2index, stepNum - i - 1);
	}
	//system("pause");
	Recover(task);
	return 0;
}

/*
void TaskExtension(Task* task, MatrixMap* map) {
	task->PushAll("allExtendedPoints");
	vector<int> sepStuckGroups;
	vector<int> moveStuckGroups;
	int depth = task->AssemblyTree.depth(task->AssemblyTree.root());
	for (int i = 0; i < depth - 1; i++) { // the leaf layer of assembly tree is not needed for extension
		// construct the task subgroups
		vector<TaskSubgroup>* taskGroups = new vector<TaskSubgroup>();
		GetTaskSubgroups(taskGroups, task->AssemblyTree.root(), task->SegTree.root(), task, 0, i);

		bool sepComplete = false;
		while (!sepComplete) {
			// separation
			for (int i = 0; i < taskGroups->size(); ++i) {
				cout << "Size: " << taskGroups->size() << "  taskGroups " << i << " : " << (*taskGroups)[i].leader << endl
					<< " separation distance: " << (*taskGroups)[i].targetSepDistance 
					<< " current distance: " << (*taskGroups)[i].currentSepDistance << endl;
				(*taskGroups)[i].Separation(map);
				//RecordTaskExtendRT(task, robot);
				task->PushAll("allExtendedPoints");
				if ((*taskGroups)[i].sepStuck)
					sepStuckGroups.push_back(i);  // update sepsStucks
			}
			
			// EndCheck: if all sepDone, complete; otherwise, move
			sepComplete = true;
			for (int i = 0; i < taskGroups->size(); ++i) {
				if (!(*taskGroups)[i].sepDone) { sepComplete = false; break; }
			}
			cout << "Separation proess: " << sepComplete << endl;
			cout << "sepStuckGroups:  ";
			for (int i = 0; i < sepStuckGroups.size(); i++)
				cout << sepStuckGroups[i] << " , ";
			cout << endl;
			if (!sepComplete) {
				// check sepStuckGroups, and assign weights
				AssignWeights(taskGroups, sepStuckGroups);
				// move
				for (int i = 0; i < taskGroups->size(); ++i) {
					vector<int> trial = (*taskGroups)[i].TrialMove();
					if (!(*taskGroups)[i].MoveCheck(map, trial)) {  // no collision
						(*taskGroups)[i].Move(map, trial, trial, {1, 1});
						// update flags
						(*taskGroups)[i].UpdateFlags(trial, true);
					}
					else
						(*taskGroups)[i].UpdateFlags(trial, false);
				}
				//RecordTaskExtendRT(task, robot);
				task->PushAll("allExtendedPoints");
				map->Display("task");
			}
			// update sepsStucks & moveStucks
			sepStuckGroups.swap(vector<int> ());
			moveStuckGroups.swap(vector<int>());
		}
		// display
		cout << endl << "Extend step " << i << " : " << endl;
		map->Display("task");
		cout << endl;
		task->PushAll("allTargets");
	}
}
*/


void TaskExtension(Task* task, MatrixMap* map) {
	task->PushAll("allExtendedPoints");
	vector<int> sepStuckGroups;
	vector<int> moveStuckGroups;
	int depth = task->AssemblyTree.depth(task->AssemblyTree.root());
	for (int i = 0; i < depth - 1; i++) { // the leaf layer of assembly tree is not needed for extension
		// construct the task subgroups
		vector<TaskSubgroup>* taskGroups = new vector<TaskSubgroup>();
		GetTaskSubgroups(taskGroups, task->AssemblyTree.root(), task->SegTree.root(), task, 0, i);

		bool sepComplete = false;
		vector<int> collision;
		while (!sepComplete) {
			for (int i = 0; i < taskGroups->size(); ++i) {
				if (!(*taskGroups)[i].sepDone) {
					// normal separation
					collision = (*taskGroups)[i].Separation(map);
					// if two sides are obstacle, shear separation
					if (collision[0] == 1 && collision[1] == 1) { // one direction
						collision = (*taskGroups)[i].ShearDeform(map, 1);
					}
					if (collision[0] == 1 && collision[1] == 1) { // another direction
						collision = (*taskGroups)[i].ShearDeform(map, -1);
					}
				}
				// if separation is stuck, move. at most one side is obstacle. 
				if (collision[0] && collision[1]) {
					//
				}
				// if move, clockwise move

				// cannot move, null
			}
		}




		bool sepComplete = false;
		while (!sepComplete) {
			// separation
			for (int i = 0; i < taskGroups->size(); ++i) {
				cout << "Size: " << taskGroups->size() << "  taskGroups " << i << " : " << (*taskGroups)[i].leader << endl
					<< " separation distance: " << (*taskGroups)[i].targetSepDistance
					<< " current distance: " << (*taskGroups)[i].currentSepDistance << endl;
				(*taskGroups)[i].Separation(map);
				//RecordTaskExtendRT(task, robot);
				task->PushAll("allExtendedPoints");
				if ((*taskGroups)[i].sepStuck)
					sepStuckGroups.push_back(i);  // update sepsStucks
			}

			// EndCheck: if all sepDone, complete; otherwise, move
			sepComplete = true;
			for (int i = 0; i < taskGroups->size(); ++i) {
				if (!(*taskGroups)[i].sepDone) { sepComplete = false; break; }
			}
			cout << "Separation proess: " << sepComplete << endl;
			cout << "sepStuckGroups:  ";
			for (int i = 0; i < sepStuckGroups.size(); i++)
				cout << sepStuckGroups[i] << " , ";
			cout << endl;
			if (!sepComplete) {
				// check sepStuckGroups, and assign weights
				AssignWeights(taskGroups, sepStuckGroups);
				// move
				for (int i = 0; i < taskGroups->size(); ++i) {
					vector<int> trial = (*taskGroups)[i].TrialMove();
					if (!(*taskGroups)[i].MoveCheck(map, trial)) {  // no collision
						(*taskGroups)[i].Move(map, trial, trial, {1, 1});
						// update flags
						(*taskGroups)[i].UpdateFlags(trial, true);
					}
					else
						(*taskGroups)[i].UpdateFlags(trial, false);
				}
				//RecordTaskExtendRT(task, robot);
				task->PushAll("allExtendedPoints");
				map->Display("task");
			}
			// update sepsStucks & moveStucks
			sepStuckGroups.swap(vector<int> ());
			moveStuckGroups.swap(vector<int>());
		}

		// display
		cout << endl << "Extend step " << i << " : " << endl;
		map->Display("task");
		cout << endl;
		task->PushAll("allTargets");
	}
}


void AssignWeights(vector<TaskSubgroup>* taskGroups, vector<int> sepStuckGroups) {
	// handle separation stucks
	for (int i = 0; i < sepStuckGroups.size(); ++i) {
		// separation stuck groups size
		int maxX = (*taskGroups)[sepStuckGroups[i]].lboundary[0]; 
		int minX = (*taskGroups)[sepStuckGroups[i]].rboundary[1];
		int maxY = (*taskGroups)[sepStuckGroups[i]].lboundary[2];
		int minY = (*taskGroups)[sepStuckGroups[i]].rboundary[3];
		// stuck points separation direction
		for (int j = 0; j < taskGroups->size(); ++j) {
			// initialization weights
			(*taskGroups)[j].neighborWeight.swap(vector<float>());
			for (int i = 0; i < 4; ++i)
				(*taskGroups)[j].neighborWeight.push_back(0.0);

			// helper groups size
			int maxX_helper = (*taskGroups)[j].lboundary[0];
			int minX_helper = (*taskGroups)[j].rboundary[1];
			int maxY_helper = (*taskGroups)[j].lboundary[2];
			int minY_helper = (*taskGroups)[j].rboundary[3];
			if ((*taskGroups)[sepStuckGroups[i]].segDir == 'x') {
				if (maxY_helper >= minY && minY_helper <= maxY) { // in the shot
					if (maxX_helper <= minX)
						(*taskGroups)[j].neighborWeight[3] = 1;  // move right
					else if (minX_helper >= maxX)
						(*taskGroups)[j].neighborWeight[2] = 1;  // move left
				}
			}
			else if ((*taskGroups)[sepStuckGroups[i]].segDir == 'y') {
				if (maxX_helper >= minX && minX_helper <= maxX) { // in the shot
					if (maxY_helper <= minY)
						(*taskGroups)[j].neighborWeight[1] = 1;  // move down
					else if (minY_helper >= maxY)
						(*taskGroups)[j].neighborWeight[0] = 1;  // move up
				}
			}
		}
	}
	// handle move stucks

	// normalization // free groups, no assignment
	for (int i = 0; i < taskGroups->size(); ++i) {
		float sum = accumulate((*taskGroups)[i].neighborWeight.begin(), (*taskGroups)[i].neighborWeight.end(), 0);
		cout << "ID: " << (*taskGroups)[i].leader << "Weights sum: " << sum << endl;
		if (int(sum)) { // sum != 0
			for (int j = 0; j < 4; ++j)
				(*taskGroups)[i].neighborWeight[j] = (*taskGroups)[i].neighborWeight[j] / sum;
		}
	}
	// ��Բrange+1��Χ��û����������㣬����weights=0

}

void GetTaskSubgroups(vector<TaskSubgroup>* taskGroups, BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, int depth, int obj) {
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



// assign the task to the closest robots using optimization (or bid)
// from task->allTargets[j][i]->taskpoint.x(y)
// to robot[i]->initPosition.x(y)
// return the assigned task IDs corresponding to the robot index
vector<int> AssignTaskToRobot(Task* task, vector<Robot*> robot) {
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
	RecordLog("Success to assign tasks to robots!");
	
	return assignedTaskID;
}

void RecordTaskExtendRT(Task* task, vector<Robot*> robots) {
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

// record the extended position of task points
void RecordTaskExtend(Task* task, vector<Robot*> robots) {
	vector<int> tIDs;
	for (int i = 0; i < robots.size(); ++i)
		for (int j = 0; j < task->allExtendedPoints[0].size(); ++j)
			if (task->allExtendedPoints[0][j]->id == robots[i]->taskID) {
				tIDs.push_back(j);
				break;
			}
	// tIDs���α���robot[0]-[end]�Ķ�Ӧ��allTargets��ID
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

// task ID->robot index // robot ID -> robot index
vector<vector<int>> IDtoIndex(vector<Robot*> robot) {
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

// get the peers' IDs of robot group
vector<int> GetPeers(RobotGroup group, vector<Robot*> robot, Task* task, vector<int> tID2index, int layer) {
	vector<BinNode<vector<int>>*> nodeVec = task->AssemblyTree.getLayerNode(task->AssemblyTree.root(), 0, layer, nodeVec);
	vector<int> peerIDs;
	for (int i = 0; i < nodeVec.size(); i++) { // check each group
		bool isPeer = false;
		for (int j = 0; j < nodeVec[i]->data.size(); j++) { // each ID
			int robotID = robot[tID2index[nodeVec[i]->data[j]]]->id;
			peerIDs.push_back(robotID);
			if (group.robot[0]->id == robotID) isPeer = true; // pair the group
		}
		if (isPeer) return peerIDs;
		peerIDs.swap(vector<int>());
	}
	return peerIDs;
}

// record the current position of robots
bool RecordRobotPosition(vector<Robot*> robots) {
	ofstream f;
	f.open("../TestRobot/Robot_Current_Position.txt", ofstream::out);
	if (f) {
		f << robots.size() << endl;
		RecordLog("RecordCurrentAndTargetPosition:");
		RecordLog("RobotId   CurrentPosition   TargetPosition");
		for (int i = 0; i < robots.size(); ++i) {
			f << robots[i]->id << "," << robots[i]->currentPosition.x+1 << "," << robots[i]->currentPosition.y+1 << ","
				<< robots[i]->targetPosition.x+1 << "," << robots[i]->targetPosition.y+1 << endl;
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
void Recover(Task* task) {
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

// check all the robots whether reach their targets
bool CheckReach(vector<RobotGroup> groups) {
	for (int i = 0; i < groups.size(); i++) {
		if (groups[i].robot[0]->currentPosition.x != groups[i].robot[0]->targetPosition.x
			|| groups[i].robot[0]->currentPosition.y != groups[i].robot[0]->targetPosition.y) {
			return false;
		}
	}
	return true;
}

// dock // put robots into one group
vector<RobotGroup> Dock(vector<Robot*> robot, Task* task, vector<int> tID2index, int layer) {  // layer = stepNum - i - 1
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
	// see the groups // assign the leaders 
	for (int j = 0; j < groups.size(); j++) {
		//cout << "group " << j << ":";  groups[j].Display();  cout << endl;
		groups[j].AssignLeaders();
	}
	RecordLog("Finished to dock!");
	cout << endl << "Finished to dock!" << endl;
	return groups;
}