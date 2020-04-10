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

	// for separation
	int targetSepDistance;  // target separation distance between two groups
	int currentSepDistance; // current separation distance
	int range;     // separation range between single robot
	char segDir;   // the separation line could only be x or y;
	bool sepDone;   // flag: the separation process is finished
	bool sepStuck;  // flag: the separation process is stuck
	
	// for group moving
	int moveStep;          // moving steps in one layer
	vector<bool> stuck4; // stuck state at 4 different directions, up, down, left, right
	bool moveStuck;         // flag indicates whether stuck
	vector<float> neighborWeight;  // probability of moving to 4 neighbor, normalized sum to 1, up, down, left, right

	// left
	int ltaskNumber;
	int lleader;
	vector<int> ltaskCenter;  // x,y
	vector<TaskPoint*> ltasks; // group members
	vector<int> lboundary; // max X, min X, max Y, min Y
	// right
	int rtaskNumber;
	int rleader;
	vector<int> rtaskCenter; // x,y
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
		Boundary();
		SepDistance(); // must after Boundary()
		// initialization
		InitWeights(); // initial weights and stuck flags

		srand(unsigned(time(NULL)));
	}

	void InitWeights();
	void UpdateFlags(vector<int>, bool);
	vector<int> TrialMove();
	void Move(MatrixMap* world, vector<int> trial_left, vector<int> trial_right, vector<int> permission);
	bool MoveCheck(MatrixMap* world, vector<int> trial);
	vector<int> SepCheck(MatrixMap* world, vector<int> trial_left, vector<int> trial_right);
	void Separation(MatrixMap* world);
	void SepDistance();
	void Boundary();
	vector<vector<int>> GetTaskPos(string);
	vector<int> GetTaskIds(string);
};

void TaskSubgroup::InitWeights() {
	neighborWeight.swap(vector<float>()); // leader
	for (int i = 0; i < 4; ++i)
		neighborWeight.push_back(0.0);  // up, down, left, right

	moveStuck = false;
	for (int i = 0; i < 4; ++i)
		stuck4.push_back(false);

	sepDone = false;
	sepStuck = false;
	currentSepDistance = 0;
	moveStep = 0;
	/*
	if (segDir == 'x') {
		if (LorR == 'r')   // which side
			neighborWeight[3] = 1.0;  // right
		else if (LorR == 'l')
			neighborWeight[2] = 1.0;  // left
	}
	else if (segDir == 'y') {
		if (LorR == 'r')
			neighborWeight[1] = 1.0;  // down
		else if (LorR == 'l')
			neighborWeight[0] = 1.0;  // up
	}
	*/
}

void TaskSubgroup::Boundary() {
	// left
	lboundary.push_back(0);   // max X
	lboundary.push_back(INT_MAX);   // min X
	lboundary.push_back(0);   // max Y
	lboundary.push_back(INT_MAX);   // min Y
	for (int i = 0; i < ltaskNumber; ++i) {
		if (ltasks[i]->taskPoint.x > lboundary[0]) { lboundary[0] = ltasks[i]->taskPoint.x; }
		if (ltasks[i]->taskPoint.x < lboundary[1]) { lboundary[1] = ltasks[i]->taskPoint.x; }
		if (ltasks[i]->taskPoint.y > lboundary[2]) { lboundary[2] = ltasks[i]->taskPoint.y; }
		if (ltasks[i]->taskPoint.y < lboundary[3]) { lboundary[3] = ltasks[i]->taskPoint.y; }
	}
	// right
	rboundary.push_back(0);   // max X
	rboundary.push_back(INT_MAX);   // min X
	rboundary.push_back(0);   // max Y
	rboundary.push_back(INT_MAX);   // min Y
	for (int i = 0; i < rtaskNumber; ++i) {
		if (rtasks[i]->taskPoint.x > rboundary[0]) { rboundary[0] = rtasks[i]->taskPoint.x; }
		if (rtasks[i]->taskPoint.x < rboundary[1]) { rboundary[1] = rtasks[i]->taskPoint.x; }
		if (rtasks[i]->taskPoint.y > rboundary[2]) { rboundary[2] = rtasks[i]->taskPoint.y; }
		if (rtasks[i]->taskPoint.y < rboundary[3]) { rboundary[3] = rtasks[i]->taskPoint.y; }
	}
}

void TaskSubgroup::UpdateFlags(vector<int> trial, bool OK) {  // is move OK?
	if (OK) {
		for (int i = 0; i < 4; ++i)
			stuck4[i] = false;
		moveStuck = false;
		sepStuck = false;
	}
	else if (!OK) {
		if (trial[1] == 1) stuck4[0] = true;       // up
		else if (trial[1] == -1) stuck4[1] = true; // down
		else if (trial[0] == 1) stuck4[2] = true;  // left
		else if (trial[0] == -1) stuck4[3] = true; // right

		// only if all 4 stuck is true, stuck flag is true
		moveStuck = true;
		for (int i = 0; i < 4; ++i)
			if (stuck4[i] == false) { moveStuck = false; break; }
		sepStuck = false;
	}
}

vector<int> TaskSubgroup::TrialMove() {
	//srand((int)time(0));
	float oneStep = random();
	char move = ' ';
	vector<int> trial(2, 0); // x, y

	float interval1 = neighborWeight[0];
	float interval2 = interval1 + neighborWeight[1];
	float interval3 = interval2 + neighborWeight[2];
	float interval4 = interval3 + neighborWeight[3];
	if (0 < oneStep && oneStep < interval1) { trial[1] = 1; move = 'u'; }               // up
	else if (interval1 < oneStep && oneStep < interval2) { trial[1] = -1; move = 'd'; } // down
	else if (interval2 < oneStep && oneStep < interval3) { trial[0] = 1; move = 'l'; }  // left
	else if (interval3 < oneStep && oneStep < interval4) { trial[0] = -1; move = 'r'; } // right

	for (int i = 0; i < ltaskNumber; ++i) {
		ltasks[i]->tendPosition.x = ltasks[i]->taskPoint.x + trial[0];
		ltasks[i]->tendPosition.y = ltasks[i]->taskPoint.y + trial[1];
	}
	for (int i = 0; i < rtaskNumber; ++i) {
		rtasks[i]->tendPosition.x = rtasks[i]->taskPoint.x + trial[0];
		rtasks[i]->tendPosition.y = rtasks[i]->taskPoint.y + trial[1];
	}
	cout << "Section of choice: (0, " << interval1 << "), (" << interval1 << ", " << interval2 <<
		"), (" << interval2 << ", " << interval3 << "), (" << interval3 << ", " << interval4 << ")" << endl;
	cout << "Trial move choice:  " << oneStep << ", " << move << endl;
	return trial;
}

void TaskSubgroup::Move(MatrixMap* world, vector<int> trial_left, vector<int> trial_right, vector<int> permission) {
	// map
	if (permission[0]) {
		for (int i = 0; i < ltaskNumber; ++i)  // left
			world->map_task(ltasks[i]->taskPoint.x, ltasks[i]->taskPoint.y) = 0;
	}
	if (permission[1]) {
		for (int i = 0; i < rtaskNumber; ++i)  // right
			world->map_task(rtasks[i]->taskPoint.x, rtasks[i]->taskPoint.y) = 0;
	}
	// move
	if (permission[0]) {
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
	}
	if (permission[1]) {
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
	}
}

bool TaskSubgroup::MoveCheck(MatrixMap* world, vector<int> trial) {
	// simply move, true means there is collision
	// trial_left, trial_right, the trial moves of left and right
	vector<int> ltendboundary; 
	ltendboundary.push_back(lboundary[0] + trial[0]);  // max X
	ltendboundary.push_back(lboundary[1] + trial[0]);  // min X 
	ltendboundary.push_back(lboundary[2] + trial[1]);  // max Y
	ltendboundary.push_back(lboundary[3] + trial[1]);  // min Y
	vector<int> rtendboundary;
	rtendboundary.push_back(rboundary[0] + trial[0]);  // max X
	rtendboundary.push_back(rboundary[1] + trial[0]);  // min X 
	rtendboundary.push_back(rboundary[2] + trial[1]);  // max Y
	rtendboundary.push_back(rboundary[3] + trial[1]);  // min Y

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

vector<int> TaskSubgroup::SepCheck(MatrixMap* world, vector<int> trial_left, vector<int> trial_right) {
	vector<int> OK{1, 1}; // 1 means ok to move, 0 means not ok to move // left, right
	// trial_left is {1, 0} or {0, 1}, trial_right is {-1, 0} or {0, -1} // x, y
	vector<int> ltendboundary{ lboundary[0] + trial_left[0], lboundary[1] + trial_left[0], 
		lboundary[2] + trial_left[1], lboundary[3] + trial_left[1] }; // max X, min X, max Y, min Y 
	vector<int> rtendboundary{ rboundary[0] + trial_right[0], rboundary[1] + trial_right[0],
		rboundary[2] + trial_right[1], rboundary[3] + trial_right[1] }; // max X, min X, max Y, min Y
	// secure range outside
	int minX, maxX, minY, maxY;
	ltendboundary[0] + range < world->ColNum - 1 ? maxX = ltendboundary[0] + range : maxX = world->ColNum - 1; // max X
	rtendboundary[1] - range > 0 ? minX = rtendboundary[1] - range : minX = 0;                                 // min X
	ltendboundary[2] + range < world->RowNum - 1 ? maxY = ltendboundary[2] + range : maxY = world->RowNum - 1; // max Y
	rtendboundary[3] - range > 0 ? minY = rtendboundary[3] - range : minY = 0;                                 // min Y

	if (trial_left[0] && trial_right[0]) { // x separation
		// check whether points cross the border
		if ((ltendboundary[0] > world->ColNum - 1)) { cout << "Caution: Out of left border !!!" << endl; OK[0] = 0; } // left
		if ((rtendboundary[1] < 0)) { cout << "Caution: Out of right border !!!" << endl; OK[1] = 0; } // right
		// check obstacle
		if (OK[0] && trial_left[0]) { 
			for (int y = ltendboundary[3]; y <= ltendboundary[2]; ++y) {
				if (world->map_obstacle(ltendboundary[0], y)) { cout << "Caution: obstacle at left !!!" << endl; OK[0] = 0; break; } // left most
			}
		}
		if (OK[1] && trial_right[0]) {
			for (int y = rtendboundary[3]; y <= rtendboundary[2]; ++y) {
				if (world->map_obstacle(rtendboundary[1], y)) { cout << "Caution: obstacle at right !!!" << endl; OK[1] = 0; break; } // right most
			}
		}
		// check other task points
		if (OK[0] && trial_left[0]) { // move left
			for (int y = minY; y <= maxY; ++y) {
				for (int x = maxX; x > ltendboundary[0]; --x) {  // left most
					if (world->map_task(x, y)) {
						cout << "Caution: other tasks at left !!!" << endl;
						OK[0] = 0; 
						y = maxY + 1;
						break;
					}
				}
			}
		}
		if (OK[1] && trial_right[0]) {  // move right
			for (int y = minY; y <= maxY; ++y) {
				for (int x = minX; x < rtendboundary[1]; ++x) {  // right most
					if (world->map_task(x, y)) {
						cout << "Caution: other tasks at right !!!" << endl;
						OK[1] = 0; 
						y = maxY + 1;
						break;
					}
				}
			}
		}
	}
	else if (trial_left[1] && trial_right[1]) { // y separation
		// check whether points cross the border
		if ((ltendboundary[2] > world->RowNum - 1)) { cout << "Caution: Out of up border!!!" << endl; OK[0] = 0; } // up
		if ((rtendboundary[3] < 0)) { cout << "Caution: Out of down border!!!" << endl; OK[1] = 0; } // down
		// check obstacle
		if (OK[0] && trial_left[1]) {
			for (int x = ltendboundary[1]; x <= ltendboundary[0]; ++x) {
				if (world->map_obstacle(x, ltendboundary[2])) { cout << "Caution: obstacle!!!" << endl; OK[0] = 0; break; } // up most
			}
		}
		if (OK[1] && trial_right[1]) {
			for (int x = rtendboundary[1]; x <= rtendboundary[0]; ++x) {
				if (world->map_obstacle(x, rtendboundary[3])) { cout << "Caution: obstacle!!!" << endl; OK[1] = 0; break; } // down most
			}
		}
		// check other task points
		if (OK[0] && trial_left[1] > 0) {  // move up
			for (int x = minX; x <= maxX; ++x) {
				for (int y = maxY; y > ltendboundary[2]; --y) { // up most
					if (world->map_task(x, y)) {
						cout << "Caution: other tasks above !!!" << endl;
						OK[0] = 0; 
						x = maxX + 1;
						break;
					}
				}
			}
		}
		if (OK[1] && trial_right[1] < 0) {  // move down
			for (int x = minX; x <= maxX; ++x) {
				for (int y = minY; y < rtendboundary[3]; ++y) {  // down most
					if (world->map_task(x, y)) {
						cout << "Caution: other tasks below at ( " << x << ", " << y << " ) !!!" << endl;
						OK[1] = 0; 
						x = maxX + 1;
						break;
					}
				}
			}
		}
	}
	//
	return OK;
}

void TaskSubgroup::Separation(MatrixMap* world) { // only separation, if sepStuck, explore
	while (!sepDone && !sepStuck) {  // if not done and not stuck
		// trial separation move
		int delta_x = 0;
		int delta_y = 0;
		if (segDir == 'x') delta_x = 1;
		else if (segDir == 'y') delta_y = 1;
		for (int i = 0; i < ltaskNumber; ++i) {
			ltasks[i]->tendPosition.x = ltasks[i]->taskPoint.x + delta_x;
			ltasks[i]->tendPosition.y = ltasks[i]->taskPoint.y + delta_y;
		}
		for (int i = 0; i < rtaskNumber; ++i) {
			rtasks[i]->tendPosition.x = rtasks[i]->taskPoint.x - delta_x;
			rtasks[i]->tendPosition.y = rtasks[i]->taskPoint.y - delta_y;
		}
		// check collision
		vector<int> trial_left = { delta_x, delta_y };
		vector<int> trial_right = { -1 * delta_x, -1 * delta_y };
		// move the two subgroups for one step
		vector<int> permission = SepCheck(world, trial_left, trial_right);
		cout << "Trial move of left: " << trial_left[0] << " , " << trial_left[1] << endl;
		cout << "Trial move of right: " << trial_right[0] << " , " << trial_right[1] << endl;
		cout << "Collision check: " << permission[0] << " , " << permission[1] << endl;
		if (permission[0] || permission[1]) {  // at least one part faces no collision
			cout << "Real move: " << endl;
			Move(world, trial_left, trial_right, permission);
			// Endcheck, update the flags: sepDone, sepStuck
			currentSepDistance += accumulate(permission.begin(), permission.end(), 0);
			if (currentSepDistance >= targetSepDistance)
				sepDone = true;
			sepStuck = false;
		 }
		else {    // collision
			sepStuck = true;
		}
	}
}

void TaskSubgroup::SepDistance() {  //  calculate the separation distance
	int lwidth = 0;
	int rwidth = 0;
	if (segDir == 'x') {
		lwidth = lboundary[0] - lboundary[1];  // left
		rwidth = rboundary[0] - rboundary[1];  // right
	}
	else if (segDir == 'y') {
		lwidth = lboundary[2] - lboundary[3];  // left
		rwidth = rboundary[2] - rboundary[3];  // right
	}
	//cout << "Left boundary:  " << lboundary[0] << " , " << lboundary[1] << " , " << lboundary[2] << " , " << lboundary[3] << endl;
	//cout << "Right boundary:  " << rboundary[0] << " , " << rboundary[1] << " , " << rboundary[2] << " , " << rboundary[3] << endl;
	//cout << "Width:  " << lwidth << " , " << rwidth << endl;
	targetSepDistance = int((float(lwidth) / 2 + float(rwidth) / 2 + 1) * float(range));
}

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

/*
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
*/

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

/*
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
		//cout << "stuck points size:  " << stuckPoints.size() << endl;
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
*/
