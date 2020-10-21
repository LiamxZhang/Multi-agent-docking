#pragma once
// 定义世界地图，包括障碍物，机器人，任务点
// 作为环境类，对机器人的行为和请求做出反应

#include <Eigen/Dense>
#include <fstream>

#include "Point.h"
#include "Log.h"

#define LogFlag false

using namespace Eigen;

class MatrixMap {
public:
	// functions
	MatrixMap() : RowNum(0), ColNum(0) {} // 行为
	MatrixMap(int row, int col) : RowNum(row), ColNum(col) {
		map_obstacle = MatrixXi::Zero(row, col); map_robot = MatrixXi::Zero(row, col); map_task = MatrixXi::Zero(row, col); }

	void ReadMap(string data_dir);
	void Display(string str); 
	int TaskCheck(vector<Point> positions, vector<int> trial, vector<int> memberIDs, vector<int> peerIDs, int interval);
	bool CollisionCheck(vector<Point> positions, vector<int> ids, vector<int> peerIDs, vector<Point> addObstacles = vector<Point> (), int interval = 1);  // check collision with robots and obstacles
	vector<int> FindWhere(int, char); // find the position according to ID
	vector<vector<Point>> Clustering(vector<Point>taskPos);
	bool IsOneGroup(vector<Point>taskPos);
	vector<Point> FindConnectTaskGroup(vector<Point> newGroup, vector<Point> candidateGroup, vector<Point> allTaskPos);
	// variables
	int RowNum, ColNum;
	MatrixXi map_obstacle; // 障碍物地图，自由点为0，障碍物为1
	MatrixXi map_robot;// 机器人地图，自由点为0，机器人点为其ID， ID 不能为0
	MatrixXi map_task;// 任务点地图，自由点为0，任务点为其ID
	

};

void
MatrixMap::ReadMap(string data_dir) {
	ifstream f;
	// read the obstacle map
	f.open(data_dir+"InitMap.txt", ifstream::in);
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
		if (LogFlag)
			RecordLog("Read map: Failed to open the InitMap.txt!");
		return;
	}
	f.close();

	// read the robot map
	f.open(data_dir+"Task.txt", ifstream::in);
	if (f) {
		int taskNum;
		f >> taskNum;
		//TaskPoint tempTask;
		for (int i = 0; i < taskNum; i++) {
			int id, x, y;
			char c;   // to not accept the comma
			f >> id >> c >> x >> c >> y;
			map_task(x - 1, y - 1) = id;
		}
		//TaskToDo = ToDoTask;
	}
	else {
		cout << "Read map: Failed to open the task file . Please check the filename ." << endl;
		if (LogFlag)
			RecordLog("Read map: Failed to read task map!");
		return;
	}
	f.close();
	// read the task map
	f.open(data_dir+"Robot_Init_Position.txt", ifstream::in);
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
		if (LogFlag)
			RecordLog("Read map: Failed to read task map!");
		return;
	}
	f.close();
	cout << "Success to read the Map of obstacle, robot and task." << endl;
	if (LogFlag)
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

int
MatrixMap::TaskCheck(vector<Point> positions, vector<int> trial, vector<int> memberIDs, vector<int> peerIDs, int interval) { // a group of robots // false : no collision, true : collision
	// return flags indicating no obstacle (0), obstacles/border (1) or other points (2)
	vector<int> intervals = { interval, interval, interval, interval };
	if (trial[0] == 1 && trial[1] == 0) { intervals[0] = 0; } // left
	if (trial[0] == -1 && trial[1] == 0) { intervals[1] = 0; } // right
	if (trial[0] == 0 && trial[1] == 1) { intervals[2] = 0; } // up
	if (trial[0] == 0 && trial[1] == -1) { intervals[3] = 0; } // down

	for (int i = 0; i < positions.size(); ++i) {  // for each robot
		if (positions[i].x < 0 || positions[i].x > RowNum - 1 || positions[i].y < 0 || positions[i].y > ColNum - 1) // out of border
			return 1;

		if (map_obstacle(positions[i].x, positions[i].y)) // obstacles
			return 1;

		if (map_task(positions[i].x, positions[i].y)) { // first only check peers
			vector<int>::iterator iter = find(peerIDs.begin(), peerIDs.end(), map_task(positions[i].x, positions[i].y));
			if (iter != peerIDs.end()) return 2;
		}

		// if there's no obstacle, no peers, check other robots at nearby
		int minX, maxX, minY, maxY;
		(positions[i].x - intervals[0] > 0) ? minX = positions[i].x - intervals[0] : minX = 0;
		(positions[i].x + intervals[1] < ColNum - 1) ? maxX = positions[i].x + intervals[1] : maxX = ColNum - 1;
		(positions[i].y - intervals[2] > 0) ? minY = positions[i].y - intervals[2] : minY = 0;
		(positions[i].y + intervals[3] < RowNum - 1) ? maxY = positions[i].y + intervals[3] : maxY = RowNum - 1;

		for (int x = minX; x <= maxX; ++x) {
			for (int y = minY; y <= maxY; ++y) {
				if (map_task(x, y)) { // robot exists
					vector<int>::iterator iter1 = find(memberIDs.begin(), memberIDs.end(), map_task(x, y));
					vector<int>::iterator iter2 = find(peerIDs.begin(), peerIDs.end(), map_task(x, y));
					if (iter1 == memberIDs.end() && iter2 == peerIDs.end()) // not member neither peer
						return 2;
				}
			}
		}
	}
	return 0;
}

bool
MatrixMap::CollisionCheck(vector<Point> positions, vector<int> memberIDs, vector<int> peerIDs, vector<Point> addObstacles, int interval) { // a group of robots // false : no collision, true : collision
	for (int i = 0; i < positions.size(); ++i) {  // for each robot
		for (int j = 0; j < addObstacles.size(); ++j) {// gate obstacles
			if (positions[i].x == addObstacles[j].x && positions[i].y == addObstacles[j].y)
				return true;
		}
		if (positions[i].x < 0 || positions[i].x > RowNum - 1 || positions[i].y < 0
			|| positions[i].y > ColNum - 1) // out of border
			return true;
		if (map_obstacle(positions[i].x, positions[i].y)) // obstacles
			return true;
		if (map_robot(positions[i].x, positions[i].y)) { // peers: to be docked
			vector<int>::iterator iter = find(peerIDs.begin(), peerIDs.end(), map_robot(positions[i].x, positions[i].y));
			if (iter != peerIDs.end()) return true;
		}
		// if there's no obstacle, no peers, check other robots at nearby
		int minX, maxX, minY, maxY;
		(positions[i].x - interval > 0) ? minX = positions[i].x - interval : minX = 0;
		(positions[i].x + interval < ColNum - 1) ? maxX = positions[i].x + interval : maxX = ColNum - 1;
		(positions[i].y - interval > 0) ? minY = positions[i].y - interval : minY = 0;
		(positions[i].y + interval < RowNum - 1) ? maxY = positions[i].y + interval : maxY = RowNum - 1;
		/*
		for (int x = minX; x <= maxX; ++x) {
			for (int y = minY; y <= maxY; ++y) {
				if (map_robot(x, y)) { // robot exists
					vector<int>::iterator iter1 = find(memberIDs.begin(), memberIDs.end(), map_robot(x,y));
					vector<int>::iterator iter2 = find(peerIDs.begin(), peerIDs.end(), map_robot(x, y));
					if (iter1 == memberIDs.end() && iter2 == peerIDs.end()) // not member neither peer
						return true;
				}
			}
		}
		*/
		//
		for (int x = minX; x <= maxX; ++x) {
			if (map_robot(x, positions[i].y)) { // robot exists
				vector<int>::iterator iter1 = find(memberIDs.begin(), memberIDs.end(), map_robot(x, positions[i].y));
				vector<int>::iterator iter2 = find(peerIDs.begin(), peerIDs.end(), map_robot(x, positions[i].y));
				if (iter1 == memberIDs.end() && iter2 == peerIDs.end()) // not member neither peer
					return true;
			}
		}

		for (int y = minY; y <= maxY; ++y) {
			if (map_robot(positions[i].x, y)) { // robot exists
				vector<int>::iterator iter1 = find(memberIDs.begin(), memberIDs.end(), map_robot(positions[i].x, y));
				vector<int>::iterator iter2 = find(peerIDs.begin(), peerIDs.end(), map_robot(positions[i].x, y));
				if (iter1 == memberIDs.end() && iter2 == peerIDs.end()) // not member neither peer
					return true;
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

vector<vector<Point>> 
MatrixMap::Clustering(vector<Point> taskPos) {
	vector<vector<Point>> candidateGroups; 
	vector<Point> consideredPos;

	for (int i = 0; i < taskPos.size(); ++i) {
		vector<Point> newCandidate;
		vector<Point>::iterator ite = find(consideredPos.begin(), consideredPos.end(), taskPos[i]);
		if (ite == consideredPos.end()) { // have not been considered
			vector<Point> newGroup;
			newGroup.push_back(taskPos[i]);
			newCandidate = FindConnectTaskGroup(newGroup, newGroup, taskPos);
			if (newCandidate.size()) {
				candidateGroups.push_back(newCandidate);
				consideredPos.insert(consideredPos.end(), newCandidate.begin(), newCandidate.begin());
			}
			
		}

	}

	return candidateGroups;
}

bool 
MatrixMap::IsOneGroup(vector<Point>taskPos) {
	int count = 0;
	vector<Point> consideredPos;

	for (int i = 0; i < taskPos.size(); ++i) {
		vector<Point>::iterator ite = find(consideredPos.begin(), consideredPos.end(), taskPos[i]);
		if (ite == consideredPos.end()) { // have not been considered
			vector<Point> newGroup;
			newGroup.push_back(taskPos[i]);
			vector<Point> newCandidate = FindConnectTaskGroup(newGroup, newGroup, taskPos);
			if (newCandidate.size() == taskPos.size())
				return true;
			else
				return false;

			if (newCandidate.size()) {
				consideredPos.insert(consideredPos.end(), newCandidate.begin(), newCandidate.begin());
				count++;
			}
		}
		if (count > 1) 
			return false;
	}

	return true;
}


// recursion, return the connectted group which the candidate group belongs to
vector<Point>
MatrixMap::FindConnectTaskGroup(vector<Point> newGroup, vector<Point> candidateGroup, vector<Point> allTaskPos) {
	// newGroup is to find the new points, candidateGroup is the newly formed group, allTaskPos is the all considered group
	// end condition
	if (!newGroup.size()) return candidateGroup;

	// 1. find the neighbors // in the allTaskPos but not in the candidateGroup
	vector<Point> news;
	for (int i = 0; i < newGroup.size(); ++i) {
		int x = newGroup[i].x;
		int y = newGroup[i].y;
		
		if ((x + 1) < (RowNum - 1) && map_task(x + 1, y)) {
			Point newPoint = Point(x + 1, y);
			vector<Point>::iterator ite1 = find(candidateGroup.begin(), candidateGroup.end(), newPoint);
			vector<Point>::iterator ite2 = find(allTaskPos.begin(), allTaskPos.end(), newPoint);
			if (ite1 == candidateGroup.end() && ite2 != allTaskPos.end()) {
				// in allTaskPos but not in candidateGroup
				news.push_back(newPoint);
				candidateGroup.push_back(newPoint);
			}
		}
		if ((x - 1) > 0 && map_task(x - 1, y)) {
			Point newPoint = Point(x - 1, y);
			vector<Point>::iterator ite1 = find(candidateGroup.begin(), candidateGroup.end(), newPoint);
			vector<Point>::iterator ite2 = find(allTaskPos.begin(), allTaskPos.end(), newPoint);
			if (ite1 == candidateGroup.end() && ite2 != allTaskPos.end()) {
				news.push_back(newPoint);
				candidateGroup.push_back(newPoint);
			}
		}
		if ((y + 1) < (ColNum - 1) && map_task(x, y + 1)) {
			Point newPoint = Point(x, y + 1);
			vector<Point>::iterator ite1 = find(candidateGroup.begin(), candidateGroup.end(), newPoint);
			vector<Point>::iterator ite2 = find(allTaskPos.begin(), allTaskPos.end(), newPoint);
			if (ite1 == candidateGroup.end() && ite2 != allTaskPos.end()) {
				news.push_back(newPoint);
				candidateGroup.push_back(newPoint);
			}
		}
		if ((y - 1) > 0 && map_task(x, y - 1)) {
			Point newPoint = Point(x, y - 1);
			vector<Point>::iterator ite1 = find(candidateGroup.begin(), candidateGroup.end(), newPoint);
			vector<Point>::iterator ite2 = find(allTaskPos.begin(), allTaskPos.end(), newPoint);
			if (ite1 == candidateGroup.end() && ite2 != allTaskPos.end()) {
				news.push_back(newPoint);
				candidateGroup.push_back(newPoint);
			}
		}
		
	}

	// 2. call self
	candidateGroup = FindConnectTaskGroup(news, candidateGroup, allTaskPos);
}