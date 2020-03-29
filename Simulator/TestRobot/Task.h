#pragma once
// TaskPoint定义单个任务点的数据格式
// 任务点的ID 与 机器人的ID 匹配
// Task读取任务，生成任务
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <numeric>
#include <cstdlib>
#include <ctime>

#include "Map.h"
#include "BinTree.h"
#include "Point.h"
#include "Log.h"

#define random() (rand() / double(RAND_MAX))

using namespace std;

class TaskPoint {            // class TaskPoint to describe the ponit details of the task
public:
	TaskPoint() :id(0), taskPoint(), stuck(false) {}
	TaskPoint(int i, int x, int y) :id(i), stuck(false) {
		taskPoint.x = x; taskPoint.y = y;
		tendPosition.x = x; tendPosition.y = y;
	}
	bool operator == (const TaskPoint& r) const {
		return (id == r.id) &&
			(taskPoint == r.taskPoint);
	}

	// variables
	int id;            // task point id
	int step;          // steps in one layer
	Point taskPoint;     // point information
	Point tendPosition;  // one step of trial move
	vector<bool> stuck4; // stuck state at different directions, up, down, left, right
	bool stuck;         // flag indicates stuck state
	bool complete;
	vector<float> neighborWeight;  // probability of moving to 4 neighbor, normalized sum to 1, up, down, left, right

	friend int main();
	friend class Rule;
	friend class Robot;
	friend class Task;
};


class TaskSubgroup {
public:
	// variables
	int taskNumber;
	int leader;    // leaer ID, while index always be 0
	vector<int> taskCenter;
	vector<TaskPoint*> tasks; // group members

	// functions
	TaskSubgroup(vector<TaskPoint*> t) : taskNumber(t.size()), tasks(t) {
		if (t.size()) leader = t[0]->id; // assign the leader ID
		srand(unsigned(time(NULL)));
	}
	TaskSubgroup(vector<TaskPoint*> t, vector<int> center, char segDir, char LorR) : 
		taskNumber(t.size()), tasks(t), taskCenter(center) {
		if (t.size()) {
			leader = t[0]->id; // assign the leader ID
			InitWeights(segDir, LorR);
		}
	}
	void InitWeights(char segDir, char LorR);
	void UpdateStuck(char);
	char TrialMove();
	void Move(MatrixMap* world);
	bool EndCheck(MatrixMap* world, int range);
	vector<vector<int>>  GetTaskPos();
	vector<int> GetTaskIds();
};

void TaskSubgroup::InitWeights(char segDir, char LorR) {
	tasks[0]->neighborWeight.swap(vector<float>()); // leader
	for (int j = 0; j < 4; ++j)
		tasks[0]->neighborWeight.push_back(0.0);  // up, down, left, right
	if (segDir == 'x') {
		if (LorR == 'r')   // which side
			tasks[0]->neighborWeight[3] = 1.0;  // right
		else if (LorR == 'l')
			tasks[0]->neighborWeight[2] = 1.0;  // left
	}
	else if (segDir == 'y') {
		if (LorR == 'r')
			tasks[0]->neighborWeight[1] = 1.0;  // down
		else if (LorR == 'l')
			tasks[0]->neighborWeight[0] = 1.0;  // up
	}
}

void TaskSubgroup::UpdateStuck(char dir) {
	int n = 0;
	switch (dir) {
	case 'u': n = 0; break;
	case 'd': n = 1; break;
	case 'l': n = 2; break;
	case 'r': n = 3; break;
	}
	for (int i = 0; i < taskNumber; ++i) {
		tasks[i]->stuck4[n] = true;
		// only if all 4 stuck is true, stuck flag is true
		tasks[i]->stuck = true;
		for (int j = 0; j < 4; ++j) {
			if (!tasks[i]->stuck4[j]) {
				tasks[i]->stuck = false;
				break;
			}
		}
	}

}

char TaskSubgroup::TrialMove() {
	//srand((int)time(0));
	float oneStep = random();
	// leader is tasks[0]
	float interval1 = tasks[0]->neighborWeight[0];
	float interval2 = interval1 + tasks[0]->neighborWeight[1];
	float interval3 = interval2 + tasks[0]->neighborWeight[2];
	float interval4 = interval3 + tasks[0]->neighborWeight[3];
	char move = ' ';
	if (0 < oneStep && oneStep < interval1) {   // up
		for (int i = 0; i < taskNumber; ++i) {
			tasks[i]->tendPosition.y = tasks[i]->taskPoint.y + 1;
		}
		move = 'u';
	}
	else if (interval1 < oneStep && oneStep < interval2) {  // down
		for (int i = 0; i < taskNumber; ++i) {
			tasks[i]->tendPosition.y = tasks[i]->taskPoint.y - 1;
		}
		move = 'd';
	}
	else if (interval2 < oneStep && oneStep < interval3) {  // left
		for (int i = 0; i < taskNumber; ++i) {
			tasks[i]->tendPosition.x = tasks[i]->taskPoint.x + 1;
		}
		move = 'l';
	}
	else if (interval3 < oneStep && oneStep < interval4) {  // right
		for (int i = 0; i < taskNumber; ++i) {
			tasks[i]->tendPosition.x = tasks[i]->taskPoint.x - 1;
		}
		move = 'r';
	}
	cout << "Section of choice: (0, " << interval1 << "), (" << interval1 << ", " << interval2 <<
		"), (" << interval2 << ", " << interval3 << "), (" << interval3 << ", " << interval4 << ")" << endl;
	cout << "Trial move choice:  " << oneStep << ", " << move << endl;
	return move;
}

void TaskSubgroup::Move(MatrixMap* world) {
	// map
	for (int i = 0; i < taskNumber; ++i) 
		world->map_task(tasks[i]->taskPoint.x, tasks[i]->taskPoint.y) = 0;
	for (int i = 0; i < taskNumber; ++i) {
		// map
		world->map_task(tasks[i]->tendPosition.x, tasks[i]->tendPosition.y) = tasks[i]->id;
		// move
		tasks[i]->taskPoint.x = tasks[i]->tendPosition.x;
		tasks[i]->taskPoint.y = tasks[i]->tendPosition.y;
		// update the stuck flag
		tasks[i]->stuck = false;
		for (int j = 0; j < 4; ++j)
			tasks[i]->stuck4[j] = false;
	}
}

bool TaskSubgroup::EndCheck(MatrixMap* world, int range) {
	vector<int> peerIDs = GetTaskIds();
	// check neighbor
	bool complete = true;
	for (int i = 0; i < taskNumber; ++i) {
		// tasks[i]
		int minX, maxX, minY, maxY;
		int X = tasks[i]->taskPoint.x;
		int Y = tasks[i]->taskPoint.y;
		X - range > 0 ? minX = X - range : minX = 0;
		X + range < world->ColNum - 1 ? maxX = X + range : maxX = world->ColNum - 1;
		Y - range > 0 ? minY = Y - range : minY = 0;
		Y + range < world->RowNum - 1? maxY = Y + range : maxY = world->RowNum - 1;

		int Y_array[2] = { minY, maxY };
		for (int x = minX; x <= maxX; ++x) {
			for (int& y : Y_array) {         // up, down
				if (world->map_task(x, y)) {
					vector<int>::iterator it = find(peerIDs.begin(), peerIDs.end(), world->map_task(x, y));
					if (it == peerIDs.end()) {   // not in component
						complete = false;
						x = maxX + 1;
						break;
					}
				}
			}
		}
		if (!complete) break;
		//
		int X_array[2] = { minX, maxX };
		for (int y = minY + 1; y < maxY; ++y) {
			for (int& x : X_array) {         // left, right
				if (world->map_task(x, y)) {
					vector<int>::iterator it = find(peerIDs.begin(), peerIDs.end(), world->map_task(x, y));
					if (it == peerIDs.end()) {   // not in component
						complete = false;
						y = maxY;
						break;
					}
				}
			}
		}
		if (!complete) break;
	}
	// assign complete
	for (int i = 0; i < taskNumber; ++i)
		tasks[i]->complete = complete;
	return complete;
}

vector<vector<int>>  TaskSubgroup::GetTaskPos() {
	// collect the tendPosition (x,y)
	vector<vector<int>> taskPos;
	for (int i = 0; i < tasks.size(); ++i) {
		vector<int> oneTask;
		oneTask.push_back(tasks[i]->tendPosition.x);
		oneTask.push_back(tasks[i]->tendPosition.y);
		taskPos.push_back(oneTask);
	}
	return taskPos;
}

vector<int>  TaskSubgroup::GetTaskIds() {
	vector<int> taskIDs;
	for (int i = 0; i < tasks.size(); ++i) {
		taskIDs.push_back(tasks[i]->id);
	}
	return taskIDs;
}


class Task {
public:
	Task() : taskNum(0), robotNum(0), startPoints(), finalTargets() { SegNum.push_back(0); SegNum.push_back(0);}

	// variables
	int taskNum;
	int robotNum;
	vector<int> SegNum;   // total times of segmentation in direction (x,y)
	vector<int> centerPoint;   // center of all task points (x,y)
	vector<vector<int>> stuckPoints;    // all stuck points (x,y)
	vector<TaskPoint*> startPoints;       // starting points of task , i.e., the robots
	vector<TaskPoint*> finalTargets;     // end points of task
	vector<TaskPoint*> currentTargets;       // current and temporary task points during the extension, IDs are identical to finalTargets
	vector<vector<TaskPoint*>> allTargets; // ids and positions of targets points for every step division, saved from currentTargets
	vector<vector<TaskPoint*>> allExtendedPoints; // record all process
	BinTree<vector<int>> AssemblyTree; // data is the robot IDs
	BinTree<char> SegTree;  //  record the segmentation line for every component
	bool ExtensionComplete; // extension complete flag
	// functions
	void Initialization();
	bool ReadTask();
	bool GenerateTree();
	void Bisect(BinNode<vector<int>>*, BinNode<char>*);
	bool GetNode();
	void PushAll(string);    // Store the currentTargets in allTargets or allExtendedPoints;

	void Display(string);      // display task positions of steps
	void Display(int);
	bool UpdateTaskmap(MatrixMap*, int);
	void UpdateWeightAndFlag();

private:
};

void 
Task::Initialization() {
	for (int i = 0; i < taskNum; ++i) {
		// step
		currentTargets[i]->step = 0;
		// complete flag
		currentTargets[i]->complete = false;
		// stuck flag
		currentTargets[i]->stuck = false;
		currentTargets[i]->stuck4.swap(vector<bool>());
		for (int j = 0; j < 4; ++j)
			currentTargets[i]->stuck4.push_back(false);
		// weights
		currentTargets[i]->neighborWeight.swap(vector<float>());
	}
	ExtensionComplete = false;  // overal complete flag
	stuckPoints.swap(vector<vector<int>>());  // clear the stuckPoints
}

bool
Task::ReadTask() {
	ifstream f;                // Task.txt 中第一行为目标目标数目，后面每行为ID和坐标
	////read the final target points
	f.open("../TestRobot/Task.txt", ifstream::in);
	if (f) {
		f >> taskNum;
		//TaskPoint tempTask;
		int task_id, task_x, task_y;
		int x_sum = 0;
		int y_sum = 0;
		for (int i = 0; i < taskNum; i++) {
			f >> task_id >> task_x >> task_y;
			TaskPoint* target = new TaskPoint(task_id, task_x - 1, task_y - 1);
			finalTargets.push_back(target);
			TaskPoint* temp = new TaskPoint(task_id, task_x - 1, task_y - 1);
			currentTargets.push_back(temp);
			x_sum += task_x - 1;
			y_sum += task_y - 1;
		}
		allTargets.push_back(finalTargets);
		//BinNode<vector<TaskPoint*>>* root = allTargets.insertASRoot(finalTargets);
		centerPoint.push_back(x_sum / taskNum);
		centerPoint.push_back(y_sum / taskNum);
	}
	else {
		cout << "Read task: Failed to open the task file. Please check the filename." << endl;
		RecordLog("Read task: Failed to open the task file. Please check the filename.");
	}
	f.close();
	
	//// read the start points
	f.open("../TestRobot/Robot_Init_Position.txt", ifstream::in);
	if (f) {
		f >> robotNum;
		if (robotNum != taskNum) {
			cout << "The robot NO. is not equal to task NO. Please check the robot NO.!" << endl;
			RecordLog("The robot NO. is not equal to task NO. Please check the robot NO.!");
		}
		int robot_id, robot_x, robot_y;
		for (int i = 0; i < robotNum; ++i) {
			int tempdata;
			char tempchar;
			f >> tempdata;
			robot_id = tempdata;
			f >> tempchar;  // ","
			f >> tempdata;
			robot_x = tempdata;  // 
			f >> tempchar;  // ","
			f >> tempdata;
			robot_y = tempdata;
			TaskPoint* start = new TaskPoint(robot_id, robot_x - 1, robot_y - 1);
			startPoints.push_back(start);
			//cout << " start points:" << task_x << " and " << task_y;
		}
	}
	else {
		cout << "Read task: Failed to open the Robot_Init_Position file. Please check the filename." << endl;
		RecordLog("Read task: Failed to open the Robot_Init_Position file. Please check the filename.");
	}
	f.close();
	cout << "Finish to read the task and the robot initial position." << endl;
	RecordLog("Finish to read the task and the robot initial position.");
	return true;
}

void
Task::Bisect(BinNode<vector<int>>* node, BinNode<char>* segNode) {
	if (!node) return;
	vector<int> components = node->data;  // components (robot group)是 node中包含的targets的ID
	int compSize = components.size();
	if (compSize <= 1) return;  // components 元素仅一个时，停止分割
	vector<int> ids;   // 找到ID对应的targets在finalTargets中的位置，存入ids中
	for (int i = 0; i < compSize; i++)   
		for (int j = 0; j < finalTargets.size(); j++)   
			if (finalTargets[j]->id == components[i])    // ids[i] <-> components[i]
				ids.push_back(j);
	
	//cout << "Component size: " << compSize << endl;
	/*
	cout << "components:  ";
	for (const int& k : components)
		cout << k << "  ";
	cout << endl;
	*/
	// initialization
	int max = 0;
	int line = 0;
	char xory = ' ';	
	for (int i = 0; i < compSize; i++) {   // every possible split line // find id == components[j] as split line
		int countX = 0;
		int countY = 0;
		for (int j = 0; j < compSize; j++) {
			// split line is x = finalTargets[i].taskPoint.x
			// from id == components[j] find finalTargets
			int id_pos;
			if (finalTargets[ids[j]]->taskPoint.x > finalTargets[ids[i]]->taskPoint.x)
				countX++;
			// split line is y = finalTargets[i].taskPoint.y
			if (finalTargets[ids[j]]->taskPoint.y > finalTargets[ids[i]]->taskPoint.y)
				countY++;
		}
		// comparison for the maximum
		if (countX * (compSize - countX) > max) {  // x direction
			max = countX * (compSize - countX);
			line = ids[i];  
			xory = 'x';
		}
		if (countY * (compSize - countY) > max) {  // y direction
			max = countY * (compSize - countY);
			line = ids[i];
			xory = 'y';
		}
	}
	// save the ids into the tree nodes
	vector<int> left;
	vector<int> right;
	if (xory == 'x') {
		//cout << "Split line: x = " << finalTargets[line]->taskPoint.x << endl;
		for (int i = 0; i < compSize; i++) {
			if (finalTargets[ids[i]]->taskPoint.x > finalTargets[line]->taskPoint.x)
				left.push_back(components[i]);
			else right.push_back(components[i]);
		}
		SegNum[0] += 1;  // x
	}
	else {
		//cout << "Split line: y = " << finalTargets[line]->taskPoint.y << endl;
		for (int i = 0; i < compSize; i++) {
			if (finalTargets[ids[i]]->taskPoint.y > finalTargets[line]->taskPoint.y)
				left.push_back(components[i]);
			else right.push_back(components[i]);
		}
		SegNum[1] += 1;  // y
	}
	node->lChild = new BinNode<vector<int>>(left);
	node->rChild = new BinNode<vector<int>>(right);

	segNode->data = xory;
	segNode->lChild = new BinNode<char>('a'); // a means nonsense
	segNode->rChild = new BinNode<char>('a');

	/*
	cout << "left:  ";
	for (const int& k : left)
		cout << k << " ";
	cout << endl << "right:  ";
	for (const int& k : right)
		cout << k << " ";
	cout << endl;
	*/
	//
	Bisect(node->lChild, segNode->lChild);
	Bisect(node->rChild, segNode->rChild);
}

bool
Task::GenerateTree() {
	// Initially, all ids into root
	vector<int> ids;
	for (int i = 0; i < taskNum; i++) ids.push_back(finalTargets[i]->id);
	if (ids.size() == 0) {
		cout << "Generate tree: Task NO. is 0. Failed to generate the assembly tree!" << endl;
		RecordLog("Generate tree: Task NO. is 0. Failed to generate the assembly tree!");
		return false;
	}
	else {
		BinNode<vector<int>>* root = AssemblyTree.insertASRoot(ids);
		BinNode<char>* segRoot = SegTree.insertASRoot('x');
		Bisect(root, segRoot);
		AssemblyTree.DisplayTree();
		cout << "Flattern Tree:" << endl;
		SegTree.display(SegTree.root());
		cout << endl;
		RecordLog("Success to generate the assembly tree!");
		return true;
	}
}

bool
Task::GetNode() {
	return true;
}

void 
Task::PushAll(string toWhere) {  // after extending task in each loop
	vector<TaskPoint*> tempTargets;
	for (int i = 0; i < taskNum; i++) {
		TaskPoint* temp = new TaskPoint();
		temp->id = currentTargets[i]->id;
		temp->taskPoint.x = currentTargets[i]->taskPoint.x;
		temp->taskPoint.y = currentTargets[i]->taskPoint.y;
		tempTargets.push_back(temp);
	}
	if (toWhere == "allTargets") {
		allTargets.push_back(tempTargets);
	}
	else if (toWhere == "allExtendedPoints") {
		allExtendedPoints.push_back(tempTargets);
	}
}

void 
Task::Display(string str) {
	if (str == "all") {
		for (int i = 0; i < allTargets.size(); i++) {
			cout << "Target step " << i << ": ";  // i denotes the ith step, j denotes the jth robot
			for (int j = 0; j < robotNum; j++)
				cout << "(" << allTargets[i][j]->id << ",  " << allTargets[i][j]->taskPoint.x
				<< ", " << allTargets[i][j]->taskPoint.y << "), ";
			cout << endl;
		}
	}
	else {
		cout << "Display input parameter is wrong!" << endl;
	}
}

void
Task::Display(int step) {
	cout << endl << "allTargets  " << step << ": ";
	for (int j = 0; j < allTargets[step].size(); j++) {
		cout << "(" << allTargets[step][j]->id << ", " << allTargets[step][j]->taskPoint.x << ", "
			<< allTargets[step][j]->taskPoint.y << "),";
	}
	cout << endl;
}

bool 
Task::UpdateTaskmap(MatrixMap* world, int num) {
	vector<TaskPoint*> newTasks = allTargets[num];
	int Size = newTasks.size();
	if (Size < 1) return false;

	// new a map
	world->map_task = MatrixXi::Zero(world->RowNum, world->ColNum);
	for (int i = 0; i < Size; i++)
		world->map_task(newTasks[i]->taskPoint.x, newTasks[i]->taskPoint.y) = newTasks[i]->id;
	return true;
}

void 
Task::UpdateWeightAndFlag() {
	// update the stuck points
	stuckPoints.swap(vector<vector<int>>());
	for (int i = 0; i < taskNum; ++i) {
		if (currentTargets[i]->stuck) {
			vector<int> temp;
			temp.push_back(currentTargets[i]->taskPoint.x);
			temp.push_back(currentTargets[i]->taskPoint.y);
			cout << "stuckPoints: " << currentTargets[i]->taskPoint.x << ", " << currentTargets[i]->taskPoint.y << endl;
			stuckPoints.push_back(temp);
		}
	}
	// update the complete flag 
	ExtensionComplete = true;
	for (int i = 0; i < taskNum; ++i) {
		if (!currentTargets[i]->complete) {
			ExtensionComplete = false;
			break;
		}
	}
	// update the step
	for (int i = 0; i < taskNum; ++i)
		currentTargets[i]->step += 1;

	// update the probability
	for (int i = 0; i < taskNum; ++i) {
		currentTargets[i]->neighborWeight.swap(vector<float> ());
		cout << "stuck points size:  " << stuckPoints.size() << endl;
		if (currentTargets[i]->complete && stuckPoints.size()) { // complete, stuck points
			for (int j = 0; j < 4; ++j)
				currentTargets[i]->neighborWeight.push_back(1.0);
			for (int j = 0; j < stuckPoints.size(); ++j) {
				if (stuckPoints[j][1] > currentTargets[i]->taskPoint.y)  // y
					currentTargets[i]->neighborWeight[0] = 0.0;  // up
				else
					currentTargets[i]->neighborWeight[1] = 0.0;  // down

				if (stuckPoints[j][0] > currentTargets[i]->taskPoint.x)   // x
					currentTargets[i]->neighborWeight[2] = 0.0;  // left
				else
					currentTargets[i]->neighborWeight[3] = 0.0;  // right
			}
		}
		else if (currentTargets[i]->complete && !stuckPoints.size()) {
			// two cases for all 0 weights: move complete & stuck points all round
			for (int j = 0; j < 4; ++j)
				currentTargets[i]->neighborWeight.push_back(0.0);
		}
		else {                           // not complete, self stuck
			for (int j = 0; j < 4; ++j)
				currentTargets[i]->neighborWeight.push_back(1.0);
			for (int j = 0; j < 4; ++j) {
				if (currentTargets[i]->stuck4[j])
					currentTargets[i]->neighborWeight[j] = 0.0;
			}
		}
		float sum = accumulate(currentTargets[i]->neighborWeight.begin(), currentTargets[i]->neighborWeight.end(), 0.0);
		if (sum) {
			for (int j = 0; j < 4; ++j)
				currentTargets[i]->neighborWeight[j] = currentTargets[i]->neighborWeight[j] / sum;
		}
	}
	
}