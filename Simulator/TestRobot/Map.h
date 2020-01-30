#pragma once
// 定义世界地图，包括障碍物，机器人，任务点
// 作为环境类，对机器人的行为和请求做出反应

#include <Eigen/Dense>
#include <fstream>

//#include "Point.h"
#include "Log.h"

using namespace Eigen;

class MatrixMap {
public:
	// functions
	MatrixMap() : RowNum(0), ColNum(0) {} // 行为
	void ReadMap();
	void Display() {
		cout << endl << "Obstacle Map: " << endl;
		//cout << map_obstacle << endl;
		cout << endl << "Robot Map: " << endl;
		//cout << map_robot << endl;
		cout << endl << "Task Map: " << endl;
		cout << map_task << endl;
	}
	bool UpdateTaskmap(vector<TaskPoint*>);  // after every move, update the Map.
	bool RobotCheck(int ID, vector<int> component, int range); // check collision with robots and obstacles
	bool TaskCheck(int, int, vector<int>, int);
	vector<int> findWhere(int, char); // find the position according to ID
	// variables
	int RowNum, ColNum;
	MatrixXi map_obstacle; // 障碍物地图，自由点为0，障碍物为1
	MatrixXi map_robot;// 机器人地图，自由点为0，机器人点为其ID， ID 不能为0
	MatrixXi map_task;// 任务点地图，自由点为0，任务点为其ID
	

};

void
MatrixMap::ReadMap() {
	ifstream f;
	// read the obstacle map
	f.open("../TestRobot/InitMap.txt", ifstream::in);
	if (f) {
		char c;
		f >> RowNum >> c >> ColNum;
		// construct map
		map_obstacle = MatrixXi::Zero(RowNum, ColNum);
		map_robot = MatrixXi::Zero(RowNum, ColNum);
		map_task = MatrixXi::Zero(RowNum, ColNum);
		for (int r = 0; r < RowNum; r++) {
			for (int c = 0; c < ColNum; c++) {
				int onepoint;
				char t;
				f >> onepoint;
				if (!onepoint) ;
				else
					map_obstacle(r,c) = onepoint;
				if (c != (ColNum - 1)) // ColNum - 1 is the last number without comma
					f >> t;//t may be the next-line signal
			}
		}
	}
	else {
		cout << " Failed to open the InitMap.txt! " << endl;
		RecordLog("Failed to open the InitMap.txt!");
		return;
	}
	f.close();

	// read the robot map
	f.open("../TestRobot/Task.txt", ifstream::in);
	if (f) {
		int taskNum;
		f >> taskNum;
		//TaskPoint tempTask;
		for (int i = 0; i < taskNum; i++) {
			int id, x, y;
			f >> id >> x >> y;
			map_task(x, y) = id;
		}
		//TaskToDo = ToDoTask;
	}
	else {
		cout << "Failed to open the task file . Please check the filename ." << endl;
		RecordLog("Failed to read task map!");
		return;
	}
	f.close();
	// read the task map
	f.open("../TestRobot/Robot_Init_Position.txt", ifstream::in);
	if (f) {
		int roboNum;
		f >> roboNum;
		//TaskPoint tempTask;
		for (int i = 0; i < roboNum; i++) {
			int id, x, y;
			char c;   // to not accept the comma
			f >> id >> c >> x >> c >> y;
			map_robot(x, y) = id;
		}
		//TaskToDo = ToDoTask;
	}
	else {
		cout << "Failed to open the task file . Please check the filename ." << endl;
		RecordLog("Failed to read task map!");
		return;
	}
	f.close();
	RecordLog("Success to read the Map of obstacle, robot and task.");
}

bool 
MatrixMap::RobotCheck(int ID, vector<int> component, int range) {
	// range 是探测的范围
	// 找到ID对应的robot位置  row, col
	int row, col;  
	for (int i = 0; i < RowNum; i++)
		for (int j = 0; j < ColNum; j++) {
			if (map_robot(i, j) == ID) {
				row = i;
				col = j;
				i = RowNum; // 跳出外层循环
				break;
			}
		}
	// 检查obstacle map
	if (map_obstacle(row, col))
		return false;
	// 检查robot map
	int rowMax, rowMin, colMax, colMin;
	if (row - range > 0) rowMin = row - range;  // 改？：语句
	else rowMin = 0;
	if (row + range < RowNum) rowMax = row + range;
	else rowMax = RowNum;
	if (col - range > 0) colMin = col - range;
	else colMin = 0;
	if (col + range < ColNum) colMax = col + range;
	else colMax = ColNum;
	for (int i = rowMin; i < rowMax; i++)
		for (int j = colMin; j < colMax; j++) {
			if (map_robot(i, j) > 0) {
				vector<int>::iterator it = find(component.begin(), component.end(), map_robot(i, j));
				if (it == component.end()) {   // 不在component中
					return false;
				}
			}			
		}
	return true;
}

bool
MatrixMap::TaskCheck(int row, int col, vector<int> component, int range) {
	// row, col是探测起点, map从0，0开始
	// component是免检的同组点
	// range 是探测的范围
	// 检查obstacle map
	if (map_obstacle(row, col))
		return false;
	// 检查task map
	int rowMax, rowMin, colMax, colMin;
	row - range > 0 ? rowMin = row - range : rowMin = 0;
	row + range < RowNum ? rowMax = row + range : rowMax = RowNum;
	col - range > 0 ? colMin = col - range : colMin = 0;
	col + range < ColNum ? colMax = col + range : colMax = ColNum;
	for (int i = rowMin; i <= rowMax; i++)
		for (int j = colMin; j <= colMax; j++) {
			//cout << "row and column：   " << i << "    " << j << endl;
			if (map_task(i, j) > 0) {
				vector<int>::iterator it = find(component.begin(), component.end(), map_task(i, j));
				if (it == component.end()) {   // 不在component中
					return false;
				}
			}
		}
	return true;
}

bool 
MatrixMap::UpdateTaskmap(vector<TaskPoint*> newTasks) {
	int Size = newTasks.size();
	if (Size < 1) return false;

	// new a map
	map_task = MatrixXi::Zero(RowNum, ColNum);
	for (int i = 0; i < Size; i++) 
		map_task(newTasks[i]->taskPoint.x, newTasks[i]->taskPoint.y) = newTasks[i]->id;
	return true;
	/*
	for (int i = 0; i < Size; i++) {
		// find the id position
		cout << "ID:" << newTasks[i]->id << endl;
		vector<int> position = findWhere(newTasks[i]->id, 't');
		// add the targets into map
		//cout << "Original position: " << position[0] << "   " << position[1] << "   " << map_task(position[0], position[1]) << endl;
		//cout << "New position: " << newTasks[i]->taskPoint.x << "   " << newTasks[i]->taskPoint.y << "  " << newTasks[i]->id << endl;
		
		map_task(position[0], position[1]) = 0;
		map_task(newTasks[i]->taskPoint.x, newTasks[i]->taskPoint.y) = newTasks[i]->id;
	}
	*/
}

vector<int>
MatrixMap::findWhere(int ID, char RorT) {
	// RorT denotes robot or task
	MatrixXi Map;
	if (RorT == 'r')
		Map = map_robot;
	else if (RorT == 't')
		Map = map_task;
	vector<int> position;
	cout << endl << Map << endl;
	for (int i = 0; i < RowNum; i++)
		for (int j = 0; j < ColNum; j++) 
			if (int(Map(i, j)) == ID) {
				position.push_back(i);
				position.push_back(j);
				return position;
			}
	// if these doesnot exist, return wrong numbers
	cout << "Error: Cannot find: " << ID << endl;
	position.push_back(RowNum+1);
	position.push_back(ColNum+1);
	return position;
}