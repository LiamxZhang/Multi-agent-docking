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
#include <algorithm>

#include "Map.h"
#include "BinTree.h"
#include "Point.h"
#include "Log.h"

#define random() (rand() / double(RAND_MAX))

using namespace std;

class TaskPoint {            // class TaskPoint to describe the ponit details of the task
public:
	TaskPoint() :id(0), taskPoint() {}
	TaskPoint(int i, int x, int y) :id(i) {
		taskPoint.x = x; taskPoint.y = y;
		tendPosition.x = x; tendPosition.y = y;
	}
	bool operator == (const TaskPoint& r) const {
		return (id == r.id) && (taskPoint.x == r.taskPoint.x) && (taskPoint.y == r.taskPoint.y);
	}

	// variables
	int id;            // task point id
	Point taskPoint;     // current position
	Point tendPosition;  // one step of trial move

	friend int main();
	friend class Rule;
	friend class Robot;
	friend class Task;
};

////////////////////////////////////////////////////////////////////////////////
class TaskSubgroup {  // include the two groups to be segmented (left, right)
public:
	// variables
	int taskNumber;  // total number
	int leader;    // leader ID, while index always be 0
	vector<float> taskCenter; // x,y

	// for separation
	int targetSepDistance;  // target separation distance between two groups
	int currentSepDistance; // current separation distance
	int range;      // distance between the individual points
	char segDir;   // the separation line could only be x or y;
	bool sepDone;   // flag: the separation process is finished
	
	// for group moving
	int moveStep;          // moving steps in one layer

	// left
	int ltaskNumber;
	int lleader;
	vector<float> ltaskCenter;  // x,y
	vector<TaskPoint*> ltasks; // group members
	vector<int> lboundary; // max X, min X, max Y, min Y
	// right
	int rtaskNumber;
	int rleader;
	vector<float> rtaskCenter; // x,y
	vector<TaskPoint*> rtasks;
	vector<int> rboundary; // max X, min X, max Y, min Y

	// functions
	TaskSubgroup(vector<TaskPoint*> left, vector<TaskPoint*> right, char c, int r) : 
		ltasks(left), ltaskNumber(left.size()), rtasks(right), rtaskNumber(right.size()), segDir(c), range(r) {
		taskNumber = ltaskNumber + rtaskNumber;
		if (left.size()) {
			leader = left[0]->id; // assign the leader ID
			lleader = left[0]->id;
		}
		if (right.size()) { rleader = right[0]->id; }
		targetSepDistance = range;
		Boundary('l'); 
		Boundary('r');
		InitWeights(); // initialization
		srand(unsigned(time(NULL)));
	}

	void InitWeights();
	// void UpdateFlags(vector<int>, bool);
	bool OverallMove(MatrixMap* , int);
	int Move(MatrixMap* world, vector<int> trial_left, vector<int> trial_right, vector<int> collosion);
	bool MoveCheck(MatrixMap* world, vector<int> trial);

	vector<int> SepCheck(MatrixMap* world, vector<int> trial_left, vector<int> trial_right);
	int TaskSubgroup::PartMoveCheck(MatrixMap* world, vector<int> trial, char LorR);
	vector<int> Separation(MatrixMap* world);
	vector<int> ShearDeform(MatrixMap*, int);

	int CalculateDistance();
	void Boundary(char);
	vector<vector<int>> GetTaskPos(string);
	vector<int> GetTaskIds(string);
};

void TaskSubgroup::InitWeights() {

	sepDone = false;
	currentSepDistance = 0;
	moveStep = 0;
	// initial center
	for (int i = 0; i < 2; ++i) {
		ltaskCenter.push_back(0);
		rtaskCenter.push_back(0);
		taskCenter.push_back(0);
	}
	
}

void TaskSubgroup::Boundary(char LorR) {
	// calculate the maximum and minimum of coordinates x,y for left and right task groups
	int taskNum;
	vector<TaskPoint*> tasks;
	if (LorR == 'l') { taskNum = ltaskNumber; tasks = ltasks; }
	else if (LorR == 'r') { taskNum = rtaskNumber; tasks = rtasks; }
	// update the boundary
	vector<int> boundary;
	boundary.push_back(INT_MIN);         // max X
	boundary.push_back(INT_MAX);   // min X
	boundary.push_back(INT_MIN);         // max Y
	boundary.push_back(INT_MAX);   // min Y
	for (int i = 0; i < taskNum; ++i) {
		if (tasks[i]->taskPoint.x > boundary[0]) { boundary[0] = tasks[i]->taskPoint.x; }
		if (tasks[i]->taskPoint.x < boundary[1]) { boundary[1] = tasks[i]->taskPoint.x; }
		if (tasks[i]->taskPoint.y > boundary[2]) { boundary[2] = tasks[i]->taskPoint.y; }
		if (tasks[i]->taskPoint.y < boundary[3]) { boundary[3] = tasks[i]->taskPoint.y; }
	}
	// update the center
	vector<float> center; // x, y
	center.push_back(float(boundary[0] + boundary[1]) / 2); // x
	center.push_back(float(boundary[2] + boundary[3]) / 2); // y
	// return to the entity
	if (LorR == 'l') {
		lboundary.swap(vector<int> ()); lboundary = boundary;
		ltaskCenter.swap(vector<float> ()); ltaskCenter = center;
		taskCenter.swap(vector<float>());
		taskCenter.push_back((ltaskCenter[0] + rtaskCenter[0]) / 2); // x
		taskCenter.push_back((ltaskCenter[1] + rtaskCenter[1]) / 2); // y
	}
	else if (LorR == 'r') {
		rboundary.swap(vector<int>()); rboundary = boundary;
		rtaskCenter.swap(vector<float>()); rtaskCenter = center;
		taskCenter.swap(vector<float>());
		taskCenter.push_back((ltaskCenter[0] + rtaskCenter[0]) / 2);
		taskCenter.push_back((ltaskCenter[1] + rtaskCenter[1]) / 2);
	}
}

int TaskSubgroup::CalculateDistance() {
	// calculate the interval distance between two subgroups
	// first, calculate the center point
	float Xdistance, Ydistance;
	Xdistance = abs(float(lboundary[0] + lboundary[1]) / 2 - float(rboundary[0] + rboundary[1]) / 2)
		- float(lboundary[0] - lboundary[1]) / 2 - float(rboundary[0] - rboundary[1]) / 2;
	Ydistance = abs(float(lboundary[2] + lboundary[3]) / 2 - float(rboundary[2] + rboundary[3]) / 2)
		- float(lboundary[2] - lboundary[3]) / 2 - float(rboundary[2] - rboundary[3]) / 2;
	
	if (Xdistance < 0) Xdistance = 0; // not negative
	if (Ydistance < 0) Ydistance = 0;
	currentSepDistance = Xdistance + Ydistance;

	return currentSepDistance;
}

vector<int> TaskSubgroup::Separation(MatrixMap* world) {
	// direct separation in opposite direction
	bool sepStuck = false;
	vector<int> collision; // no obstacle (0), obstacles (1) or other points (2)
	while (!sepStuck && !sepDone) {  // if not stuck and not finish 
		// trial separation move
		vector<int> trial_left(2);
		vector<int> trial_right(2);
		if (segDir == 'x') { trial_left[0] = 1; trial_right[0] = -1; }
		else if (segDir == 'y') { trial_left[1] = 1; trial_right[1] = -1; }
		
		for (int i = 0; i < ltaskNumber; ++i) {
			ltasks[i]->tendPosition.x = ltasks[i]->taskPoint.x + trial_left[0];
			ltasks[i]->tendPosition.y = ltasks[i]->taskPoint.y + trial_left[1];
		}
		for (int i = 0; i < rtaskNumber; ++i) {
			rtasks[i]->tendPosition.x = rtasks[i]->taskPoint.x + trial_right[0];
			rtasks[i]->tendPosition.y = rtasks[i]->taskPoint.y + trial_right[1];
		}

		// check collision, return the flag indicates no obstacle (0), obstacles (1) or other points (2)
		collision = SepCheck(world, trial_left, trial_right);
		cout << "Trial move of left: " << trial_left[0] << " , " << trial_left[1] << endl;
		cout << "Trial move of right: " << trial_right[0] << " , " << trial_right[1] << endl;
		cout << "Collision check: " << collision[0] << " , " << collision[1] << endl;

		// move for one step // two sides clear, one side clear, two sides obstacle
		if (!collision[0] || !collision[1]) {  // at least one side is clear
			cout << "Real move: " << endl;
			currentSepDistance += Move(world, trial_left, trial_right, collision);
			cout << "Current separation distance: " << currentSepDistance << endl << endl;
			// Endcheck, update the flags: sepDone
			if (currentSepDistance >= targetSepDistance)  // separation is complete
				sepDone = true;
		}
		else if (collision[0] && collision[1]){    // two sides are facing collision
			sepStuck = true;
		}
	}
	return collision;
}

vector<int> TaskSubgroup::SepCheck(MatrixMap* world, vector<int> trial_left, vector<int> trial_right) {
	vector<int> collosion; // left, right
	collosion.push_back(PartMoveCheck(world, trial_left, 'l'));
	collosion.push_back(PartMoveCheck(world, trial_right, 'r'));
	return collosion;
}

int TaskSubgroup::PartMoveCheck(MatrixMap* world, vector<int> trial, char LorR) {
	// return flags indicating no obstacle (0), obstacles/border (1) or other points (2)
	// trial move (x,y) is {1, 0} or {0, 1} or {-1, 0} or {0, -1} 
	vector<int> boundary;  // max X, min X, max Y, min Y 
	vector<TaskPoint*> partner; // partner of the under-checking group
	switch (LorR) { 
	case 'l': boundary = lboundary; partner = rtasks; break; 
	case 'r': boundary = rboundary; partner = ltasks; break; 
	}
	vector<int> tendboundary;
	tendboundary.push_back(boundary[0] + trial[0]);
	tendboundary.push_back(boundary[1] + trial[0]);
	tendboundary.push_back(boundary[2] + trial[1]);
	tendboundary.push_back(boundary[3] + trial[1]);
	// make sure boundary not move outside
	int minX, maxX, minY, maxY;
	tendboundary[0] + range < world->ColNum - 1 ? maxX = tendboundary[0] + range : maxX = world->ColNum - 1; // max X
	tendboundary[1] - range > 0 ? minX = tendboundary[1] - range : minX = 0;                                 // min X
	tendboundary[2] + range < world->RowNum - 1 ? maxY = tendboundary[2] + range : maxY = world->RowNum - 1; // max Y
	tendboundary[3] - range > 0 ? minY = tendboundary[3] - range : minY = 0;                                 // min Y

	int COL = 0; // collision
	if (trial[0]) { // x separation
		// check whether points cross the border
		if (!COL && (tendboundary[0] > world->ColNum - 1)) { cout << "Caution: Out of left border !!!" << endl; COL = 1; } // left most
		if (!COL && (tendboundary[1] < 0)) { cout << "Caution: Out of right border !!!" << endl; COL = 1; } // right most

		// check obstacle
		if (!COL) {  // if not cross the border
			for (int y = tendboundary[3]; y <= tendboundary[2]; ++y) {
				if (world->map_obstacle(tendboundary[0], y)) { cout << "Caution: obstacle at left !!!" << endl; COL = 1; break; } // left most
				if (world->map_obstacle(tendboundary[1], y)) { cout << "Caution: obstacle at right !!!" << endl; COL = 1; break; } // right most
			}
		}

		// check other task points
		if (!COL && trial[0] > 0) { // move left
			for (int y = minY; y <= maxY; ++y) {
				for (int x = maxX; x > tendboundary[0]; --x) {  // left most
					if (world->map_task(x, y)) {
						// check if it is the mate
						cout << "Caution: other tasks at left !!!" << endl;
						COL = 2;
						y = maxY + 1;
						break;
					}
				}
				
			}
		}
		else if (!COL && trial[0] < 0) { // move right
			for (int y = minY; y <= maxY; ++y) {
				for (int x = minX; x < tendboundary[1]; ++x) {  // right most
					if (world->map_task(x, y)) {
						// check if it is the mate
						cout << "Caution: other tasks at right !!!" << endl;
						COL = 2;
						y = maxY + 1;
						break;
					}
				}
			}
		}
	}
	else if (trial[1]) { // y separation
		// check whether points cross the border
		if (!COL && (tendboundary[2] > world->RowNum - 1)) { cout << "Caution: Out of up border!!!" << endl; COL = 1; } // up
		if (!COL && (tendboundary[3] < 0)) { cout << "Caution: Out of down border!!!" << endl; COL = 1; } // down

		// check obstacle
		if (!COL) {
			for (int x = tendboundary[1]; x <= tendboundary[0]; ++x) {
				if (world->map_obstacle(x, tendboundary[2])) { cout << "Caution: obstacle!!!" << endl; COL = 1; break; } // up most
				if (world->map_obstacle(x, tendboundary[3])) { cout << "Caution: obstacle!!!" << endl; COL = 1; break; } // down most
			}
		}

		// check other task points
		if (!COL && trial[1] > 0) {  // move up
			for (int x = minX; x <= maxX; ++x) {
				for (int y = maxY; y > tendboundary[2]; --y) { // up most
					if (world->map_task(x, y)) {
						cout << "Caution: other tasks above at ( " << x << ", " << y << " ) !!!" << endl;
						COL = 2;
						x = maxX + 1;
						break;
					}
				}
			}
		}
		if (!COL && trial[1] < 0) {  // move down
			for (int x = minX; x <= maxX; ++x) {
				for (int y = minY; y < tendboundary[3]; ++y) {  // down most
					if (world->map_task(x, y)) {
						cout << "Caution: other tasks below at ( " << x << ", " << y << " ) !!!" << endl;
						COL = 2;
						x = maxX + 1;
						break;
					}
				}
			}
		}
	}
	//
	return COL;
}

vector<int> TaskSubgroup::ShearDeform(MatrixMap* world, int direction) {
	// shear deformation separation in opposite direction; there are two directions, direction = -1 or 1
	vector<int> collision;
	if (!sepDone) {  // if not stuck and not finish 
		// trial separation move
		vector<int> trial_left(2);
		vector<int> trial_right(2);
		if (segDir == 'y') { trial_left[0] = direction; trial_right[0] = -1 * direction; }
		else if (segDir == 'x') { trial_left[1] = direction; trial_right[1] = -1 * direction; }

		for (int i = 0; i < ltaskNumber; ++i) {
			ltasks[i]->tendPosition.x = ltasks[i]->taskPoint.x + trial_left[0];
			ltasks[i]->tendPosition.y = ltasks[i]->taskPoint.y + trial_left[1];
		}
		for (int i = 0; i < rtaskNumber; ++i) {
			rtasks[i]->tendPosition.x = rtasks[i]->taskPoint.x + trial_right[0];
			rtasks[i]->tendPosition.y = rtasks[i]->taskPoint.y + trial_right[1];
		}

		// check collision, return the flag indicates no obstacle (0), obstacles (1) or other points (2)
		collision = SepCheck(world, trial_left, trial_right); // left, right
		cout << "Trial shear move of left: " << trial_left[0] << " , " << trial_left[1] << endl;
		cout << "Trial shear move of right: " << trial_right[0] << " , " << trial_right[1] << endl;
		cout << "Collision check: " << collision[0] << " , " << collision[1] << endl;

		// move for one step // two sides clear, one side clear, two sides obstacle
		if (!collision[0] || !collision[1]) {  // at least one side is clear
			Move(world, trial_left, trial_right, collision);
			// Endcheck, update the flags: sepDone
			currentSepDistance = CalculateDistance();
			cout << "Real Move: " << endl;
			cout << "Current separation distance: " << currentSepDistance << endl << endl;
			if (currentSepDistance >= targetSepDistance)  // separation is complete
				sepDone = true;
		}
		// else not move
	}
	return collision;
}

bool TaskSubgroup::OverallMove(MatrixMap* world, int direction) {
	// move in a clockwise direction 0~3
	vector<int> trial; // x, y
	vector<vector<int> > nextStep; // { {1,0},{0,1},{-1,0},{0,-1} }; // left, up, right, down
	vector<int> a(2); a[0] = 1; nextStep.push_back(a);
	vector<int> b(2); b[1] = 1; nextStep.push_back(b);
	vector<int> c(2); c[0] = -1; nextStep.push_back(c);
	vector<int> d(2); d[1] = -1; nextStep.push_back(d);

	for (int i = 0; i < 4; ++i) {
		vector<int> trial = nextStep[direction]; // trial move // x, y  
		
		if (MoveCheck(world, trial)) { // check
			vector<int> collision(2);
			int step = Move(world, trial, trial, collision); // move
			moveStep += step;
			return true;
		}
		else {
			direction += 1;
			if (direction == 4) direction = 0;
		}
	}
	return false; // 4 sides are obstacles
}

/*
for (int i = 0; i < ltaskNumber; ++i) {
		ltasks[i]->tendPosition.x = ltasks[i]->taskPoint.x + trial[0];
		ltasks[i]->tendPosition.y = ltasks[i]->taskPoint.y + trial[1];
	}
	for (int i = 0; i < rtaskNumber; ++i) {
		rtasks[i]->tendPosition.x = rtasks[i]->taskPoint.x + trial[0];
		rtasks[i]->tendPosition.y = rtasks[i]->taskPoint.y + trial[1];
	}
*/

int TaskSubgroup::Move(MatrixMap* world, vector<int> trial_left, vector<int> trial_right, vector<int> collision) {
	int step = 0; // count the move step, could be 0,1,2
	
	if (trial_left[0] == trial_right[0] && trial_left[1] == trial_right[1]) { // overall move
		if (!collision[0] && !collision[1]) { 
			// clear the map
			for (int i = 0; i < ltaskNumber; ++i)  // left
				world->map_task(ltasks[i]->taskPoint.x, ltasks[i]->taskPoint.y) = 0;
			for (int i = 0; i < rtaskNumber; ++i)  // right
				world->map_task(rtasks[i]->taskPoint.x, rtasks[i]->taskPoint.y) = 0;

			for (int i = 0; i < ltaskNumber; ++i) {   // left
				world->map_task(ltasks[i]->tendPosition.x, ltasks[i]->tendPosition.y) = ltasks[i]->id;
				ltasks[i]->taskPoint.x = ltasks[i]->tendPosition.x;
				ltasks[i]->taskPoint.y = ltasks[i]->tendPosition.y;
			}
			// update the boundary
			lboundary[0] = lboundary[0] + trial_left[0];  // max X 
			lboundary[1] = lboundary[1] + trial_left[0];  // min X
			lboundary[2] = lboundary[2] + trial_left[1];  // max Y
			lboundary[3] = lboundary[3] + trial_left[1];  // min Y

			for (int i = 0; i < rtaskNumber; ++i) {  // right
				world->map_task(rtasks[i]->tendPosition.x, rtasks[i]->tendPosition.y) = rtasks[i]->id;
				rtasks[i]->taskPoint.x = rtasks[i]->tendPosition.x;
				rtasks[i]->taskPoint.y = rtasks[i]->tendPosition.y;
			}
			// update the boundary
			rboundary[0] = rboundary[0] + trial_right[0];  // max X 
			rboundary[1] = rboundary[1] + trial_right[0];  // min X
			rboundary[2] = rboundary[2] + trial_right[1];  // max Y
			rboundary[3] = rboundary[3] + trial_right[1];  // min Y

			step += 1;
		} // otherwise not move
	}
	else { // separation move
		if (!collision[0]) {
			for (int i = 0; i < ltaskNumber; ++i)  // left // clear the map
				world->map_task(ltasks[i]->taskPoint.x, ltasks[i]->taskPoint.y) = 0;
			for (int i = 0; i < ltaskNumber; ++i) {   // left
				world->map_task(ltasks[i]->tendPosition.x, ltasks[i]->tendPosition.y) = ltasks[i]->id;
				ltasks[i]->taskPoint.x = ltasks[i]->tendPosition.x;
				ltasks[i]->taskPoint.y = ltasks[i]->tendPosition.y;
			}
			// update the boundary
			lboundary[0] = lboundary[0] + trial_left[0];  // max X 
			lboundary[1] = lboundary[1] + trial_left[0];  // min X
			lboundary[2] = lboundary[2] + trial_left[1];  // max Y
			lboundary[3] = lboundary[3] + trial_left[1];  // min Y
			step += 1;
		}
		if (!collision[1]) {
			for (int i = 0; i < rtaskNumber; ++i)  // right // clear the map
				world->map_task(rtasks[i]->taskPoint.x, rtasks[i]->taskPoint.y) = 0;
			for (int i = 0; i < rtaskNumber; ++i) {  // right
				world->map_task(rtasks[i]->tendPosition.x, rtasks[i]->tendPosition.y) = rtasks[i]->id;
				rtasks[i]->taskPoint.x = rtasks[i]->tendPosition.x;
				rtasks[i]->taskPoint.y = rtasks[i]->tendPosition.y;
			}
			// update the boundary
			rboundary[0] = rboundary[0] + trial_right[0];  // max X 
			rboundary[1] = rboundary[1] + trial_right[0];  // min X
			rboundary[2] = rboundary[2] + trial_right[1];  // max Y
			rboundary[3] = rboundary[3] + trial_right[1];  // min Y
			step += 1;
		}
	}
	return step;
}

bool TaskSubgroup::MoveCheck(MatrixMap* world, vector<int> trial){
	bool free = false; // left, right
	if (!PartMoveCheck(world, trial, 'l') && !PartMoveCheck(world, trial, 'r')) free = true;
	return free;
}

/*
bool TaskSubgroup::MoveCheck(MatrixMap* world, vector<int> trial) {
	// simply overall move, true means there is collision
	// the boundary of the trial moves of left and right //  max X, min X, max Y, min Y
	vector<int> ltendboundary = {lboundary[0] + trial[0], lboundary[1] + trial[0], lboundary[2] + trial[1], lboundary[3] + trial[1] };
	vector<int> rtendboundary = {rboundary[0] + trial[0], rboundary[1] + trial[0], rboundary[2] + trial[1], rboundary[3] + trial[1] };

	// check whether points cross the border
	if ((ltendboundary[0] > world->ColNum - 1) || (ltendboundary[1] < 0) ||
		(ltendboundary[2] > world->RowNum - 1) || (ltendboundary[3] < 0) ||
		(rtendboundary[0] > world->ColNum - 1) || (rtendboundary[1] < 0) ||
		(rtendboundary[2] > world->RowNum - 1) || (rtendboundary[3] < 0)) {
		cout << "Caution: Out of map border!!!" << endl;
		return true;
	}
	// check obstacle // check the new places
	// left
	for (int x = ltendboundary[1]; x <= ltendboundary[0]; ++x) {
		if (world->map_obstacle(x, ltendboundary[2])) { cout << "Caution: obstacle!!!" << endl; return true; } // up most
		if (world->map_obstacle(x, ltendboundary[3])) { cout << "Caution: obstacle!!!" << endl; return true; } // down most
	}
	for (int y = ltendboundary[3]; y <= ltendboundary[2]; ++y) {  
		if (world->map_obstacle(ltendboundary[0], y)) { cout << "Caution: obstacle!!!" << endl; return true; } // left most
		if (world->map_obstacle(ltendboundary[1], y)) { cout << "Caution: obstacle!!!" << endl; return true; } // right most
	}
	// right
	for (int x = rtendboundary[1]; x <= rtendboundary[0]; ++x) {
		if (world->map_obstacle(x, rtendboundary[2])) { cout << "Caution: obstacle!!!" << endl; return true; } // up most
		if (world->map_obstacle(x, rtendboundary[3])) { cout << "Caution: obstacle!!!" << endl; return true; } // down most
	}
	for (int y = rtendboundary[3]; y <= rtendboundary[2]; ++y) {
		if (world->map_obstacle(rtendboundary[0], y)) { cout << "Caution: obstacle!!!" << endl; return true; } // left most
		if (world->map_obstacle(rtendboundary[1], y)) { cout << "Caution: obstacle!!!" << endl; return true; } // right most
	}
	
	// check other task points
	int minX, maxX, minY, maxY;  // include left and right
	ltendboundary[0] + range < world->ColNum - 1 ? maxX = ltendboundary[0] + range : maxX = world->ColNum - 1;
	rtendboundary[1] - range > 0 ? minX = rtendboundary[1] - range : minX = 0;
	ltendboundary[2] + range < world->RowNum - 1 ? maxY = ltendboundary[2] + range : maxY = world->RowNum - 1;
	rtendboundary[3] - range > 0 ? minY = rtendboundary[3] - range : minY = 0;

	if (trial[1] > 0) {  // move up
		for (int x = minX; x <= maxX; ++x) {
			for (int y = maxY; y > ltendboundary[2]; --y) { // up most
				if (world->map_task(x, y)) {
					cout << "Caution: other tasks above !!!" << endl;
					return true;
				}
			}
		}
	}
	else if (trial[1] < 0) {  // move down
		for (int x = minX; x <= maxX; ++x) {
			for (int y = minY; y < rtendboundary[3]; ++y) {  // down most
				if (world->map_task(x, y)) {
					cout << "Caution: other tasks below at ( " << x << ", " << y << " ) !!!" << endl;
					return true;
				}
			}
		}
	}
	else if (trial[0] > 0) { // move left
		for (int y = minY; y <= maxY; ++y) {
			for (int x = maxX; x > ltendboundary[0]; --x) {  // left most
				if (world->map_task(x, y)) {
					cout << "Caution: other tasks at left !!!" << endl;
					return true;
				}
			}
		}
	}
	else if (trial[0] < 0) {  // move right
		for (int y = minY; y <= maxY; ++y) {
			for (int x = minX; x < rtendboundary[1]; ++x) {  // right most
				if (world->map_task(x, y)) {
					cout << "Caution: other tasks at right !!!" << endl;
					return true;
				}
			}
		}
	}
	// everything is OK
	return false;
}
*/

vector<vector<int>>  TaskSubgroup::GetTaskPos(string who) {
	// collect the tendPosition (x,y)
	vector<vector<int>> taskPos;
	if (who == "left") {
		for (int i = 0; i < ltaskNumber; ++i) {
			vector<int> oneTask;
			oneTask.push_back(ltasks[i]->tendPosition.x);
			oneTask.push_back(ltasks[i]->tendPosition.y);
			taskPos.push_back(oneTask);
		}
	}
	else if (who == "right") {
		for (int i = 0; i < rtaskNumber; ++i) {
			vector<int> oneTask;
			oneTask.push_back(rtasks[i]->tendPosition.x);
			oneTask.push_back(rtasks[i]->tendPosition.y);
			taskPos.push_back(oneTask);
		}
	}
	else if (who == "all") {
		for (int i = 0; i < ltaskNumber; ++i) {
			vector<int> oneTask;
			oneTask.push_back(ltasks[i]->tendPosition.x);
			oneTask.push_back(ltasks[i]->tendPosition.y);
			taskPos.push_back(oneTask);
		}
		for (int i = 0; i < rtaskNumber; ++i) {
			vector<int> oneTask;
			oneTask.push_back(rtasks[i]->tendPosition.x);
			oneTask.push_back(rtasks[i]->tendPosition.y);
			taskPos.push_back(oneTask);
		}
	}
	return taskPos;
}

vector<int>  TaskSubgroup::GetTaskIds(string who) {
	vector<int> taskIDs;
	if (who == "left") {
		for (int i = 0; i < ltaskNumber; ++i) {
			taskIDs.push_back(ltasks[i]->id);
		}
	}
	else if (who == "right") {
		for (int i = 0; i < rtaskNumber; ++i) {
			taskIDs.push_back(rtasks[i]->id);
		}
	}
	else if (who == "all") {
		for (int i = 0; i < ltaskNumber; ++i) {
			taskIDs.push_back(ltasks[i]->id);
		}
		for (int i = 0; i < rtaskNumber; ++i) {
			taskIDs.push_back(rtasks[i]->id);
		}
	}
	return taskIDs;
}

/////////////////////////////////////////////////////////////////////////

class Task {
public:
	Task() : taskNum(0), robotNum(0), startPoints(), finalTargets() {}

	// variables
	int taskNum;
	int robotNum;
	vector<int> centerPoint;   // center of all task points (x,y)
	vector<vector<int>> stuckPoints;    // all stuck points (x,y)
	vector<TaskPoint*> startPoints;       // starting points of task , i.e., the robots
	vector<TaskPoint*> finalTargets;     // end points of task
	vector<TaskPoint*> currentTargets;       // current and temporary task points during the extension, IDs are identical to finalTargets
	vector<vector<TaskPoint*>> allTargets; // ids and positions of targets points for every step division, saved from currentTargets
	vector<vector<TaskPoint*>> allExtendedPoints; // record all process
	BinTree<vector<int>> AssemblyTree; // data is the robot IDs
	BinTree<char> SegTree;  //  record the segmentation line for every component
	//bool ExtensionComplete; // extension complete flag

	// functions
	void Initialization();
	bool ReadTask();
	bool GenerateTree();
	void Bisect(BinNode<vector<int>>*, BinNode<char>*);
	void PushAll(string);      // Store the currentTargets in allTargets or allExtendedPoints;
	void Display(string);      // display task positions of steps
	void Display(int);
	bool UpdateTaskmap(MatrixMap*, int);
	void UpdateWeightAndFlag();

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
	}
	else {
		//cout << "Split line: y = " << finalTargets[line]->taskPoint.y << endl;
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
