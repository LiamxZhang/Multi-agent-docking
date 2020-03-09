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
	bool UpdateTaskmap(vector<TaskPoint*>);  // after every move, update the Map.
	bool TaskCheck(int, int, vector<int>, int);
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
			map_task(x, y) = id;
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
			map_robot(x, y) = id;
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
*/

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
	*/
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

bool
MatrixMap::CollisionCheck(vector<Point> positions, vector<int> ids, vector<int> peerIDs) { // a group of robots // false : no collision, true : collision
	for (int i = 0; i < positions.size(); ++i) {  // for each robot
		if (map_obstacle(positions[i].x, positions[i].y)) // obstacles
			return true;
		else { // if there's no obstacle, check robot at nearby
			bool collision;
			int minX, maxX, minY, maxY;
			(positions[i].x - 1 > 0) ? minX = positions[i].x - 1 : minX = 0;
			(positions[i].x + 1 < RowNum - 1) ? maxX = positions[i].x + 1 : maxX = RowNum - 1;
			(positions[i].y - 1 > 0) ? minY = positions[i].y - 1 : minY = 0;
			(positions[i].y + 1 < ColNum - 1) ? maxY = positions[i].y + 1 : maxY = ColNum - 1;
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