#pragma once
// 定义世界地图，包括障碍物，机器人，任务点
// 作为环境类，对机器人的行为和请求做出反应

#include <Eigen/Dense>
#include <fstream>

#include "Log.h"

using namespace Eigen;

class MatrixMap {
public:
	// functions
	MatrixMap() : RowNum(0), ColNum(0) {} // 行为
	MatrixMap(int row, int col) : RowNum(row), ColNum(col) {
		map_obstacle = MatrixXi::Zero(row, col); map_robot = MatrixXi::Zero(row, col); map_task = MatrixXi::Zero(row, col); }

	void ReadMap();
	void Display(string str); 
	bool TaskCheck(vector<vector<int>> taskPos, vector<int> taskIDs, vector<int> peerIDs, int range);
	bool CollisionCheck(vector<Point> positions, vector<int> ids, vector<int> peerIDs);  // check collision with robots and obstacles
	vector<int> FindWhere(int, char); // find the position according to ID
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
		map_obstacle = MatrixXi::Zero(RowNum, ColNum);
		map_robot = MatrixXi::Zero(RowNum, ColNum);
		map_task = MatrixXi::Zero(RowNum, ColNum);
		// construct map
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
		cout << "Read map: Failed to open the InitMap.txt! " << endl;
		RecordLog("Read map: Failed to open the InitMap.txt!");
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
			map_task(x - 1, y - 1) = id;
		}
		//TaskToDo = ToDoTask;
	}
	else {
		cout << "Read map: Failed to open the task file . Please check the filename ." << endl;
		RecordLog("Read map: Failed to read task map!");
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
			map_robot(x - 1, y - 1) = id;
		}
		//TaskToDo = ToDoTask;
	}
	else {
		cout << "Read map: Failed to open the task file . Please check the filename ." << endl;
		RecordLog("Read map: Failed to read task map!");
		return;
	}
	f.close();
	cout << "Success to read the Map of obstacle, robot and task." << endl;
	RecordLog("Success to read the Map of obstacle, robot and task.");
}

void 
MatrixMap::Display(string str) {
	if (str == "all") {
		cout << endl << "Obstacle Map: " << endl;
		cout << map_obstacle << endl;
		cout << endl << "Robot Map: " << endl;
		cout << map_robot << endl;
		cout << endl << "Task Map: " << endl;
		cout << map_task << endl;
	}
	else if (str == "obstacle") {
		cout << endl << "Obstacle Map: " << endl;
		cout << map_obstacle << endl;
	}
	else if (str == "robot") {
		cout << endl << "Robot Map: " << endl;
		cout << map_robot << endl;
	}
	else if (str == "task") {
		cout << endl << "Task Map: " << endl;
		cout << map_task << endl;
	}
	else
		cout << "Wrong input! Only 'all', 'obstacle', 'robot', 'task' are accepted!" << endl;
}

/*
bool
MatrixMap::TaskCheck(int row, int col, vector<int> ids, int range) {
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
	/*
	for (int i = rowMin; i <= rowMax; i++) {
		// 最左一列
		if (map_task(i, colMin) > 0) {
			vector<int>::iterator it = find(ids.begin(), ids.end(), map_task(i, colMin));
			if (it == ids.end()) {   // 不在component中
				return false;
			}
		}
		// 最右一列
		if (map_task(i, colMax) > 0) {
			vector<int>::iterator it = find(ids.begin(), ids.end(), map_task(i, colMax));
			if (it == ids.end())
				return false;
		}
	}
	
	for (int j = colMin + 1; j < colMax; j++) {
		// 最上一行
		if (map_task(rowMin, j) > 0) {
			vector<int>::iterator it = find(ids.begin(), ids.end(), map_task(rowMin, j));
			if (it == ids.end())
				return false;
		}
		// 最下一行
		if (map_task(rowMax, j) > 0) {
			vector<int>::iterator it = find(ids.begin(), ids.end(), map_task(rowMax, j));
			if (it == ids.end())
				return false;
		}
	}
	////
	for (int i = rowMin; i <= rowMax; i++)
		for (int j = colMin; j <= colMax; j++) {
			if (map_task(i, j) > 0) {
				vector<int>::iterator it = find(ids.begin(), ids.end(), map_task(i, j));
				if (it == ids.end())
					return false;
			}
		}
	return true;
}
*/

bool
MatrixMap::TaskCheck(vector<vector<int>> taskPos, vector<int> taskIDs, vector<int> peerIDs, int range) {
	// taskPos, taskIDs 是
	// peerIDs 是免检的同组的Child点
	// range 是探测的范围
	// 检查obstacle map, task map, edge
	// 检查边缘
	for (int i = 0; i < taskPos.size(); ++i) {
		if ((taskPos[i][0] < 0 * ColNum) || (taskPos[i][0] > 1 * ColNum) 
			|| (taskPos[i][1] < 0 * RowNum) || (taskPos[i][1] >  1 * RowNum))
			return false;
	}
	// 有障碍, false, 无障碍, true
	for (int i = 0; i < taskPos.size(); ++i) {
		if (map_obstacle(taskPos[i][0], taskPos[i][1]))
			return false;
	}
	// 检查task map
	for (int i = 0; i < taskPos.size(); ++i) {
		int minX, maxX, minY, maxY;
		int X = taskPos[i][0];
		int Y = taskPos[i][1];
		X - range > 0 ? minX = X - range : minX = 0;
		X + range < ColNum - 1 ? maxX = X + range : maxX = ColNum - 1;
		Y - range > 0 ? minY = Y - range : minY = 0;
		Y + range < RowNum - 1 ? maxY = Y + range : maxY = RowNum - 1;
		//
		int Y_array[2] = { minY, maxY };
		for (int x = minX; x <= maxX; ++x) {
			for (int& y : Y_array) {         // up, down
				if (map_task(x, y)) {
					vector<int>::iterator itt = find(taskIDs.begin(), taskIDs.end(), map_task(x, y));
					vector<int>::iterator itp = find(peerIDs.begin(), peerIDs.end(), map_task(x, y));
					if (itt == taskIDs.end() && itp == peerIDs.end()) {   // not in component
						return false;
					}
				}
			}
		}
		//
		int X_array[2] = { minX, maxX };
		for (int y = minY + 1; y < maxY; ++y) {
			for (int& x : X_array) {         // left, right
				if (map_task(x, y)) {
					vector<int>::iterator itt = find(taskIDs.begin(), taskIDs.end(), map_task(x, y));
					vector<int>::iterator itp = find(peerIDs.begin(), peerIDs.end(), map_task(x, y));
					if (itt == taskIDs.end() && itp == peerIDs.end()) {   // not in component
						return false;
					}
				}
			}
		}
	}
	return true;
	/*
	int rowmax = 0; 
	int rowmin = (1 << 31) - 1; // max of int
	int colmax = 0;
	int colmin = (1 << 31) - 1;  // max and min of task points

	for (int i = 0; i < taskPos.size(); ++i) {
		if (taskPos[i][0] > rowmax)
			rowmax = taskPos[i][0];
		if (taskPos[i][0] < rowmin)
			rowmin = taskPos[i][0];
		if (taskPos[i][1] > colmax)
			colmax = taskPos[i][1];
		if (taskPos[i][1] > colmin)
			colmin = taskPos[i][1];
	}

	int rowMax, rowMin, colMax, colMin;
	rowmin - range > 0 ? rowMin = rowmin - range : rowMin = 0;
	rowmax + range < RowNum ? rowMax = rowmax + range : rowMax = RowNum;
	colmin - range > 0 ? colMin = colmin - range : colMin = 0;
	colmax + range < ColNum ? colMax = colmax + range : colMax = ColNum;

	for (int i = rowMin; i <= rowMax; i++) {
		// 最左一列
		if (map_task(i, colMin) > 0) {
			vector<int>::iterator it = find(peerIDs.begin(), peerIDs.end(), map_task(i, colMin));
			if (it == peerIDs.end()) {   // 不在component中
				return false;
			}
		}
		// 最右一列
		if (map_task(i, colMax) > 0) {
			vector<int>::iterator it = find(peerIDs.begin(), peerIDs.end(), map_task(i, colMax));
			if (it == peerIDs.end())
				return false;
		}
	}

	for (int j = colMin + 1; j < colMax; j++) {
		// 最上一行
		if (map_task(rowMin, j) > 0) {
			vector<int>::iterator it = find(peerIDs.begin(), peerIDs.end(), map_task(rowMin, j));
			if (it == peerIDs.end())
				return false;
		}
		// 最下一行
		if (map_task(rowMax, j) > 0) {
			vector<int>::iterator it = find(peerIDs.begin(), peerIDs.end(), map_task(rowMax, j));
			if (it == peerIDs.end())
				return false;
		}
	}
	*/
	/*
	for (int i = rowMin; i <= rowMax; i++)
		for (int j = colMin; j <= colMax; j++) {
			if (map_task(i, j) > 0) {
				vector<int>::iterator it = find(ids.begin(), ids.end(), map_task(i, j));
				if (it == ids.end())
					return false;
			}
		}
	*/
}


bool
MatrixMap::CollisionCheck(vector<Point> positions, vector<int> ids, vector<int> peerIDs) { // a group of robots // false : no collision, true : collision
	for (int i = 0; i < positions.size(); ++i) {  // for each robot
		if (map_obstacle(positions[i].x, positions[i].y)) // obstacles
			return true;
		else { // if there's no obstacle, check robot at nearby
			bool collision;
			int minX, maxX, minY, maxY;
			(positions[i].x - 1 > 0) ? minX = positions[i].x - 1 : minX = 0;
			(positions[i].x + 1 < ColNum - 1) ? maxX = positions[i].x + 1 : maxX = ColNum - 1;
			(positions[i].y - 1 > 0) ? minY = positions[i].y - 1 : minY = 0;
			(positions[i].y + 1 < RowNum - 1) ? maxY = positions[i].y + 1 : maxY = RowNum - 1;
			for (int x = minX; x <= maxX; ++x) {
				for (int y = minY; y <= maxY; ++y) {
					if (map_robot(x, y) != 0) { // robot exists
						collision = true;  // assume existing robot
						if (x == positions[i].x && y == positions[i].y) { // at the center position
							for (int j = 0; j < ids.size(); ++j) {  // if one ID exists in the component，no collision
								if (ids[j] == map_robot(x, y)) {
									collision = false;
									break;
								}
							}
						}
						else { // not the center
							for (int j = 0; j < peerIDs.size(); ++j) {  // if one ID exists in the component，no collision
								if (peerIDs[j] == map_robot(x, y)) {
									collision = false;
									break;
								}
							}
						}
						if (collision) return true;
					}
				}
			}
			
		}
	}
	return false;
}

vector<int>
MatrixMap::FindWhere(int ID, char RorT) {
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