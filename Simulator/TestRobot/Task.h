#pragma once
// TaskPoint定义单个任务点的数据格式
// 任务点的ID 与 机器人的ID 匹配
// Task读取任务，生成任务
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

#include "BinTree.h"
#include "Point.h"
#include "Log.h"

using namespace std;

class TaskPoint {            // class TaskPoint to describe the ponit details of the task
public:
	TaskPoint() :id(0), publishTime(0), waitTime(0), taskPoint() {}
	TaskPoint(int i, int x, int y) :id(i), publishTime(0), waitTime(0) { 
		taskPoint.x = x; taskPoint.y = y;
	}
	bool operator == (const TaskPoint& r) const {
		return (id == r.id) &&
			(publishTime == r.publishTime) &&
			(waitTime == r.waitTime) &&
			(taskPoint == r.taskPoint);
	}
	bool operator == (const int& i) const {
		return (id == i);
	}
	friend class Rule;
	friend class Robot;
	friend class Task;
	friend int main();
	int id;            // task point id
	int publishTime;   // the time generating task
	int waitTime;      // the time next task wait
	Point taskPoint;     // point information
private:
};

class Task {
public:
	Task() : taskNum(0), robotNum(0), startPoints(), finalTargets() {}

	// variables
	int taskNum;
	int robotNum;
	vector<TaskPoint*> startPoints;       // starting points of task
	vector<TaskPoint*> finalTargets;     // end points of task
	vector<TaskPoint*> currentTargets;       // current and temporary task points, IDs are identical to finalTargets
	vector<vector<TaskPoint*>> allTargets; // ids and positions of targets points for every step division, saved from currentTargets
	BinTree<vector<int>> AssemblyTree; // data is the robot IDs
	BinTree<char> SegTree;  //  record the segmentation line for every component
	// functions
	bool ReadTask();
	bool GenerateTask();
	bool GenerateTree();
	void Bisect(BinNode<vector<int>>*, BinNode<char>*);
	bool GetNode();
	void PushAllTargets();    // Store the currentTargets in allTargets
	void Display(string);      // display task positions of steps
	void Display(int);
private:
};

bool
Task::ReadTask() {
	ifstream f;                // Task.txt 中第一行为目标目标数目，后面每行为ID和坐标
	////read the final target points
	f.open("../TestRobot/Task.txt", ifstream::in);
	if (f) {
		f >> taskNum;
		//TaskPoint tempTask;
		int task_id, task_x, task_y;
		for (int i = 0; i < taskNum; i++) {
			f >> task_id >> task_x >> task_y;
			TaskPoint* target = new TaskPoint(task_id, task_x, task_y);
			finalTargets.push_back(target);
			TaskPoint* temp = new TaskPoint(task_id, task_x, task_y);
			currentTargets.push_back(temp);
		}
		allTargets.push_back(finalTargets);
		//BinNode<vector<TaskPoint*>>* root = allTargets.insertASRoot(finalTargets);
	}
	else
		cout << "Failed to open the task file . Please check the filename ." << endl;
	f.close();
	cout << "Finish to read task" << endl;
	RecordLog("Finish to read task.");
	//// read the start points
	f.open("../TestRobot/Robot_Init_Position.txt", ifstream::in);
	if (f) {
		f >> robotNum;
		if (robotNum != taskNum){
			cout << "The robot NO. does not equal to task NO. Please check the robot NO.!" << endl;
		}
		int task_id, task_x, task_y;
		for (int i = 0; i < robotNum; ++i) {
			int tempdata;
			char tempchar;
			f >> tempdata;
			task_id = tempdata;
			f >> tempchar;  // ","
			f >> tempdata;
			task_x = tempdata;  // 
			f >> tempchar;  // ","
			f >> tempdata;
			task_y = tempdata;
			TaskPoint* start = new TaskPoint(task_id, task_x, task_y);
			startPoints.push_back(start);
			//cout << " start points:" << task_x << " and " << task_y;
		}
	}
	else
		cout << "Failed to open the Robot_Init_Position file . Please check the filename ." << endl;
	f.close();
	cout << "Finish to read the robot initial position." << endl;
	RecordLog("Finish to read task the robot initial position.");
	return true;
}

bool
Task::GenerateTask() {
	// 生成任务函数
	cout << "Please input the taskNum:" << endl;
	cin >> taskNum;
	/*ofstream f;
	int ptime = 0;
	f.open("../TestRobot/Task.txt", ofstream::out);
	if (f){
		f << taskNum << endl;
		for (int i = 1; i <= taskNum; ++i){
			Task tempTask;
			tempTask.id = taskId + 1;
			++ taskId;
			tempTask.publishTime = ptime;
			tempTask.waitTime = rand() % 3 + 2;
			do{
				tempTask.source.x = rand() % (mapRowNumber - 1) + 1;
				tempTask.source.y = rand() % mapColumnNumber;
				tempTask.target.x = rand() % (mapRowNumber - 1) + 1;
				tempTask.target.y = rand() % mapColumnNumber;
			} while (tempTask.source == tempTask.target ||
				map[tempTask.source.x][tempTask.source.y] == 1 ||
				map[tempTask.target.x][tempTask.target.y] == 1);
			ptime += tempTask.waitTime;
			f << tempTask.id << " " << tempTask.publishTime << " " <<
				tempTask.waitTime << " " << tempTask.source.x << " " <<
				tempTask.source.y << " " << tempTask.target.x << " " <<
				tempTask.target.y << endl;
		}
	}
	else
		cout << "Failed to open the target file . Please check the filename . " << endl;
	f.close();*/
	cout << "Success to generate task" << endl;
	return true;
}

void
Task::Bisect(BinNode<vector<int>>* node, BinNode<char>* segNode) {
	if (!node) return;
	vector<int> components = node->data;  // componnets是 node中包含的targets的ID
	int compSize = components.size();
	if (compSize <= 1) return;  // components 元素仅一个时，停止分割
	vector<int> ids;   // 找到ID对应的targets在finalTargets中的位置，存入ids中
	for (int i = 0; i < compSize; i++)   
		for (int j = 0; j < finalTargets.size(); j++)   
			if (finalTargets[j]->id == components[i])    // ids[i] <-> components[i]
				ids.push_back(j);
	
	//cout << "Component size: " << compSize << endl;
	cout << "components:  ";
	for (const int& k : components)
		cout << k << "  ";
	cout << endl;

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
		if (countX * (compSize - countX) > max) {
			max = countX * (compSize - countX);
			line = ids[i];  
			xory = 'x';
		}
		if (countY * (compSize - countY) > max) {
			max = countY * (compSize - countY);
			line = ids[i];
			xory = 'y';
		}
	}
	// save the ids into the tree nodes
	vector<int> left;
	vector<int> right;
	if (xory == 'x') {
		cout << "Split line: x = " << finalTargets[line]->taskPoint.x << endl;
		for (int i = 0; i < compSize; i++) {
			if (finalTargets[ids[i]]->taskPoint.x > finalTargets[line]->taskPoint.x)
				left.push_back(components[i]);
			else right.push_back(components[i]);
		}
		
	}
	else {
		cout << "Split line: y = " << finalTargets[line]->taskPoint.y << endl;
		for (int i = 0; i < compSize; i++) {
			if (finalTargets[ids[i]]->taskPoint.y > finalTargets[line]->taskPoint.y)
				left.push_back(components[i]);
			else right.push_back(components[i]);
		}
	}
	node->lChild = new BinNode<vector<int>>(left);
	node->rChild = new BinNode<vector<int>>(right);
	segNode->data = xory;
	segNode->lChild = new BinNode<char>('a'); // a means nonsense
	segNode->rChild = new BinNode<char>('a');

	cout << "left:  ";
	for (const int& k : left)
		cout << k << " ";
	cout << endl << "right:  ";
	for (const int& k : right)
		cout << k << " ";
	cout << endl;
	//
	Bisect(node->lChild, segNode->lChild);
	Bisect(node->rChild, segNode->rChild);
}

bool
Task::GenerateTree() {
	// Initially, all ids into root
	vector<int> ids;
	for (int i = 0; i < taskNum; i++) ids.push_back(finalTargets[i]->id);
	if (ids.size() == 0) return false;
	else {
		BinNode<vector<int>>* root = AssemblyTree.insertASRoot(ids);
		BinNode<char>* segRoot = SegTree.insertASRoot('x');
		Bisect(root, segRoot);
		AssemblyTree.DisplayTree();
		cout << endl;
		SegTree.display(SegTree.root());
		return true;
	}

}

bool
Task::GetNode() {
	return true;
}

void 
Task::PushAllTargets() {  // after extending task in each loop
	vector<TaskPoint*> tempTargets;
	for (int i = 0; i < taskNum; i++) {
		TaskPoint* temp = new TaskPoint();
		temp->id = currentTargets[i]->id;
		temp->taskPoint.x = currentTargets[i]->taskPoint.x;
		temp->taskPoint.y = currentTargets[i]->taskPoint.y;
		tempTargets.push_back(temp);
	}
	allTargets.push_back(tempTargets);
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