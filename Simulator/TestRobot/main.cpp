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
//void AssignWeights(vector<TaskSubgroup>* taskGroups, vector<int> sepStuckGroups);
void GetTaskSubgroups(vector<TaskSubgroup>* taskGroups, BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, int depth, int obj);
vector<int> AssignTaskToRobot(Task* task, vector<Robot*> robot);
vector<vector<int>> IDtoIndex(vector<Robot*> robot);
bool CheckReach(vector<RobotGroup> groups);
vector<RobotGroup> Dock(vector<Robot*> robot, Task* task, vector<int> tID2index, int layer);
vector<int> GetPeers(RobotGroup group, vector<Robot*> robot, Task* task, vector<int> tID2index, int layer);
//
vector<RobotGroup> NewGroups(vector<RobotGroup> groups, vector<Robot*> robot);
vector<int> AdjacentGroups(vector<RobotGroup> groups, vector<int> currentIDs, vector<int> allIDs);
vector<RobotGroup> FormGroup(vector<vector<int>> groupIDs, vector<RobotGroup> groups);
bool CheckAdjacent(vector<RobotGroup> groups);
int GroupDistance(RobotGroup group1, RobotGroup group2);
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
	AssignTaskToRobot(task, robot);
	// show the task extension process
	//RecordTaskExtend(task, robot); 
	
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
	int step = 0; // count the steps
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
				groups[j].PathPlanning(world, task->allTargets[stepNum - i - 1], peersIDs);
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
				return 0;
			}
		}
		world->Display("all");
		string str1 = "Finished to move all robots to the targets of layer ";
		string str2 = to_string(stepNum - i - 1);
		RecordLog(str1 + str2);
		cout << str1 + str2 << endl;
		//system("pause");
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
*/

/*
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
*/

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
		closestDistance = INT_MAX;   // max is initialized as infinity
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


/*
vector<int> AssignTaskToRobot(Task* task, vector<Robot*> robot) {
	// 从最接近的中心的任务点，分配机器人
	vector<int> queue = task->GetQueue();
	float closestDistance;
	float distance;   // distance between robots and tasks
	bool assigned;
	vector<int> assignedRobotID;
	for (int i = 0; i < task->taskNum; ++i) {
		// get the task position
		int taskX = task->allTargets.back()[queue[i]]->taskPoint.x;
		int taskY = task->allTargets.back()[queue[i]]->taskPoint.y;
		closestDistance = INT_MAX;   // max is initialized as infinity
		int chosed_robotID;
		for (int j = 0; j < task->robotNum; ++j) {
			// if task has been assigned, break
			assigned = false;
			for (int k = 0; k < assignedRobotID.size(); ++k)
				if (assignedRobotID[k] == robot[j]->id) {
					assigned = true;
					break;
				}
			if (assigned) continue;

			// get the robot position
			int roboX = robot[j]->initPosition.x;
			int roboY = robot[j]->initPosition.y;
			// calculate the Manhattan distance
			distance = abs(roboX - taskX) + abs(roboY - taskY);

			if (distance < closestDistance) {
				closestDistance = distance;
				chosed_robotID = j;
			}
		}
		// assign task to robot
		robot[chosed_robotID]->taskID = task->allTargets.back()[queue[i]]->id;
		assignedRobotID.push_back(chosed_robotID);
	}
	return assignedRobotID;
}
*/

/*
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
*/

// record the extended position of task points
/*
void RecordTaskExtend(Task* task, vector<Robot*> robots) {
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
*/

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

// get the peers' IDs in the same robot group
vector<int> GetPeers(RobotGroup group, vector<Robot*> robot, Task* task, vector<int> tID2index, int layer) {
	//vector<BinNode<vector<int>>*> nodeVec = task->AssemblyTree.getLayerNode(task->AssemblyTree.root(), 0, layer, nodeVec);
	vector<int> peerIDs;
	for (int i = 0; i < group.robotNumber; ++i) { // check each group
		int robotID = group.robot[i]->id;
		peerIDs.push_back(robotID);
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


// return the robot IDs to be join in the same group
vector<RobotGroup> NewGroups(vector<RobotGroup> groups, vector<Robot*> robot) {
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
vector<int> AdjacentGroups(vector<RobotGroup> groups, vector<int> currentIDs, vector<int> allIDs) {
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
vector<RobotGroup> FormGroup(vector<vector<int>> groupIDs, vector<RobotGroup> groups) { // group IDs
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
bool CheckAdjacent(vector<RobotGroup> groups) {
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

// check all the robots whether reach their targets
bool CheckReach(vector<RobotGroup> groups) {
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

int GroupDistance(RobotGroup group1, RobotGroup group2) {
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



// dock // put robots into one group
/*
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
*/

/*
vector<RobotGroup> Dock_Adjoin(vector<RobotGroup> groups) {
	vector<RobotGroup> old_groups = groups;
	vector<RobotGroup> new_groups;
	// loop through the robot group
	bool isAdjacent = true;
	while (isAdjacent) {
		isAdjacent = false;
		vector<int> already;
		new_groups.swap(vector<RobotGroup> ());
		for (int i = 0; i < old_groups.size(); ++i) {
			// seek its adjacent groups
			vector<int> neighbors;
			for (int j = i + 1; j < old_groups.size(); ++j) {
				// already formed in a group
				if (std::find(already.begin(), already.end(), j) != already.end())
					continue;

				else if (GroupDistance(old_groups[i], old_groups[j]) == 1) {
					//
					isAdjacent = true;
					neighbors.push_back(j);
					already.push_back(j);
				}
			}
			// i forms the new group with its neighbors
			if (neighbors.size()) {
				vector<Robot*> current_robots = old_groups[i].robot;
				for (int j = 0; j < neighbors.size(); ++j) {
					current_robots.insert(current_robots.end(), 
						old_groups[neighbors[j]].robot.begin(), 
						old_groups[neighbors[j]].robot.end());
				}
				// create the group;
				RobotGroup temp_group(current_robots);
				new_groups.push_back(temp_group);
			}
		}
		old_groups.assign(new_groups.begin(), new_groups.end());

	}

	return old_groups;
}

*/
