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
#define LogFlag false

using namespace std;

// TaskPoint

#pragma region TaskPoint

class TaskPoint {            // class TaskPoint to describe the ponit details of the task
public:
	TaskPoint() :id(0), step(0), taskPoint() {}
	TaskPoint(int i, int x, int y) :id(i), step(0) {
		taskPoint.x = x; taskPoint.y = y;
		tendPosition.x = x; tendPosition.y = y;
	}
	bool operator == (const TaskPoint& r) const {
		return (id == r.id) && (taskPoint.x == r.taskPoint.x) && (taskPoint.y == r.taskPoint.y);
	}

	// variables
	int id;            // task point id
	int step;
	Point taskPoint;     // current position
	Point tendPosition;  // one step of trial move
	Point lastPosition;  // position of last time

	friend int main();
	friend class Rule;
	friend class Robot;
	friend class Task;
};

#pragma endregion

// TaskSubgroup

#pragma region TaskSubgroup

class TaskSubgroup {  // include the two groups to be segmented (left, right)
public:
	// variables
	int taskNumber;  // total number
	int leader;    // leader ID, while index always be 0
	vector<float> taskCenter; // x,y

	// for separation
	int targetSepDistance;  // target separation distance between two groups
	int currentSepDistance; // current separation distance
	int range;      // detection distance of the individual points
	char segDir;   // the separation line could only be x or y;
	bool sepDone;   // flag: the separation process is finished
	
	// for group moving
	int moveStep;          // moving steps in one layer
	vector<float> neighborWeight;  // probability of moving to 4 neighbor, normalized sum to 1, up, down, left, right

	// left
	int ltaskNumber;
	int lleader;
	vector<float> ltaskCenter;  // x,y
	vector<float> llastPos;  // the last task centers (x,y) of left
	vector<float> ltendPos;  // the next task centers (x,y) of left
	vector<TaskPoint*> ltasks; // group members
	vector<int> lboundary; // max X, min X, max Y, min Y
	// right
	int rtaskNumber;
	int rleader;
	vector<float> rtaskCenter; // x,y
	vector<float> rlastPos;  // the last task centers (x,y) of left
	vector<float> rtendPos;  // the next task centers (x,y) of left
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
		InitWeights(); // initialization
		Boundary('l'); 
		Boundary('r');
		
		srand(unsigned(time(NULL)));
	}

	void InitWeights();
	void UpdateWeights(MatrixMap* world, int direction);
	void UpdateLastPos();
	// void UpdateFlags(vector<int>, bool);
	vector<int> TrialMove();
	void RecoverTendPos();
	bool OverallMove(MatrixMap*, int direction = 0);
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
	// initial weight
	neighborWeight.swap(vector<float>());
	for (int i = 0; i < 4; ++i)
		neighborWeight.push_back(0.0);  // up, down, left, right
	// initial tend and next centers	
	for (int i = 0; i < 2; ++i) {
		ltaskCenter.push_back(0.0);
		rtaskCenter.push_back(0.0);
		llastPos.push_back(0.0);
		ltendPos.push_back(0.0);
		rlastPos.push_back(0.0);
		rtendPos.push_back(0.0);
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
		taskCenter.push_back(float(ltaskCenter[0] + rtaskCenter[0]) / 2); // x
		taskCenter.push_back(float(ltaskCenter[1] + rtaskCenter[1]) / 2); // y
	}
	else if (LorR == 'r') {
		rboundary.swap(vector<int>()); rboundary = boundary;
		rtaskCenter.swap(vector<float>()); rtaskCenter = center;
		taskCenter.swap(vector<float>());
		taskCenter.push_back(float(ltaskCenter[0] + rtaskCenter[0]) / 2);
		taskCenter.push_back(float(ltaskCenter[1] + rtaskCenter[1]) / 2);
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

// 

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
		cout << "Separation of left: " << trial_left[0] << " , " << trial_left[1] << endl;
		cout << "Separation of right: " << trial_right[0] << " , " << trial_right[1] << endl;
		cout << "Collision check of Separation: " << collision[0] << " , " << collision[1] << endl;

		// move for one step // two sides clear, one side clear, two sides obstacle
		if (!collision[0] || !collision[1]) {  // at least one side is clear
			
			currentSepDistance += Move(world, trial_left, trial_right, collision);
			cout << "After real separation: Current separation distance: " << currentSepDistance << endl << endl;
			// Endcheck, update the flags: sepDone
			if (currentSepDistance >= targetSepDistance)  // separation is complete
				sepDone = true;
			sepStuck = false;
		}
		else if (collision[0] && collision[1]){    // two sides are facing collision
			RecoverTendPos();
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
	case 'l': if (!ltaskNumber) return 0;  boundary = lboundary; partner = rtasks; break;
	case 'r': if (!rtaskNumber) return 0; boundary = rboundary; partner = ltasks; break;
	}
	vector<int> pID;
	for (int i = 0; i < partner.size(); ++i) pID.push_back(partner[i]->id);
	
	vector<int> tendboundary; // max X, min X, max Y, min Y 
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
	if (trial[0] > 0) { // move left separation
		// check whether points cross the border
		if (!COL && (tendboundary[0] > world->ColNum - 1)) { 
			cout << "Caution: Out of left border !!!" << endl; COL = 1; 
		} // left most

		// check obstacle
		if (!COL) {  // if not cross the border
			for (int y = tendboundary[3]; y <= tendboundary[2]; ++y) {
				if (world->map_obstacle(tendboundary[0], y) || world->map_task(tendboundary[0], y)) {
					cout << "Caution: obstacle at left !!!" << endl; COL = 1; break; 
				} // left most
			}
		}

		// check other task points
		if (!COL) { // move left
			for (int y = minY; y <= maxY; ++y) {
				for (int x = maxX; x > tendboundary[0]; --x) {  // left most
					if (world->map_task(x, y)) {
						vector<int>::iterator ite = find(pID.begin(), pID.end(), world->map_task(x, y)); // check if it is the mate
						if (ite == pID.end()) {
							cout << "Caution: other tasks at left !!!" << endl;
							COL = 2;
							y = maxY + 1;
							break;
						}
					}
				}
			}
		}
		
	}
	else if (trial[0] < 0) { // move right separation
		// check whether points cross the border
		if (!COL && (tendboundary[1] < 0)) {
			cout << "Caution: Out of right border !!!" << endl; COL = 1;
		} // right most

		// check obstacle
		if (!COL) {  // if not cross the border
			for (int y = tendboundary[3]; y <= tendboundary[2]; ++y) {
				if (world->map_obstacle(tendboundary[1], y) || world->map_task(tendboundary[1], y)) {
					cout << "Caution: obstacle at right !!!" << endl; COL = 1; break; 
				} // right most
			}
		}

		// check other task points
		if (!COL) { // move right
			for (int y = minY; y <= maxY; ++y) {
				for (int x = minX; x < tendboundary[1]; ++x) {  // right most
					if (world->map_task(x, y)) {
						vector<int>::iterator ite = find(pID.begin(), pID.end(), world->map_task(x, y)); // check if it is the mate
						if (ite == pID.end()) {
							cout << "Caution: other tasks at right !!!" << endl;
							COL = 2;
							y = maxY + 1;
							break;
						}
					}
				}
			}
		}
	}
	else if (trial[1] > 0) { // y separation
		// check whether points cross the border
		if (!COL && (tendboundary[2] > world->RowNum - 1)) { cout << "Caution: Out of up border!!!" << endl; COL = 1; } // up
		
		// check obstacle
		if (!COL) {
			for (int x = tendboundary[1]; x <= tendboundary[0]; ++x) {
				if (world->map_obstacle(x, tendboundary[2]) || world->map_task(x, tendboundary[2])) {
					cout << "Caution: obstacle!!!" << endl; COL = 1; break; 
				} // up most
			}
		}

		// check other task points
		if (!COL) {  // move up
			for (int x = minX; x <= maxX; ++x) {
				for (int y = maxY; y > tendboundary[2]; --y) { // up most
					if (world->map_task(x, y)) {
						vector<int>::iterator ite = find(pID.begin(), pID.end(), world->map_task(x, y)); // check if it is the mate
						if (ite == pID.end()) {
							cout << "Caution: other tasks above at ( " << x << ", " << y << " ) !!!" << endl;
							COL = 2;
							x = maxX + 1;
							break;
						}
					}
				}
			}
		}
		
	}
	else if (trial[1] < 0) {
		// check whether points cross the border
		if (!COL && (tendboundary[3] < 0)) { cout << "Caution: Out of down border!!!" << endl; COL = 1; } // down
		
		// check obstacle
		if (!COL) {
			for (int x = tendboundary[1]; x <= tendboundary[0]; ++x) {
				if (world->map_obstacle(x, tendboundary[3]) || world->map_task(x, tendboundary[3])) {
					cout << "Caution: obstacle!!!" << endl; COL = 1; break; 
				} // down most
			}
		}

		// check other task points
		if (!COL) {  // move down
			for (int x = minX; x <= maxX; ++x) {
				for (int y = minY; y < tendboundary[3]; ++y) {  // down most
					if (world->map_task(x, y)) {
						vector<int>::iterator ite = find(pID.begin(), pID.end(), world->map_task(x, y)); // check if it is the mate
						if (ite == pID.end()) {
							cout << "Caution: other tasks below at ( " << x << ", " << y << " ) !!!" << endl;
							COL = 2;
							x = maxX + 1;
							break;
						}
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

//

void TaskSubgroup::UpdateWeights(MatrixMap* world, int direction) {
	vector<int> trial(2); // trial move
	neighborWeight.swap(vector<float>()); // clear neightWeight

	vector<float> weights;
	for (int i = 0; i < 4; ++i) weights.push_back(1.0);
	if (direction) weights[direction - 1] = 2.0; // direction: 1 up, 2 down, 3 left, 4 right, 0 no direction
	float zero = 0.0;
	float FourSideFree = accumulate(weights.begin(), weights.end(), zero);

	trial[1] = 1; // up [0,1]
	if (MoveCheck(world, trial)) neighborWeight.push_back(weights[0]);
	else neighborWeight.push_back(zero);
	trial[1] = -1; // down [0,-1]
	if (MoveCheck(world, trial)) neighborWeight.push_back(weights[1]);
	else neighborWeight.push_back(zero);
	trial[1] = 0;
	trial[0] = 1; // left [1,0]
	if (MoveCheck(world, trial)) neighborWeight.push_back(weights[2]);
	else neighborWeight.push_back(zero);
	trial[0] = -1; // right [-1,0]
	if (MoveCheck(world, trial)) neighborWeight.push_back(weights[3]);
	else neighborWeight.push_back(zero);

	float sum = accumulate(neighborWeight.begin(), neighborWeight.end(), zero);
	if (sum) {
		if (sum == FourSideFree) { // 4 sides are free, no need to move
			for (int i = 0; i < 4; ++i)
				neighborWeight[i] = zero;  // up, down, left, right
		}
		else { // 
			for (int i = 0; i < 4; ++i)
				neighborWeight[i] = neighborWeight[i] / sum; // normalize the weights
		}
	}
	cout << "Group " << leader << " weights: \t" << neighborWeight[0] << " , \t"
		<< neighborWeight[1] << " , \t" << neighborWeight[2] << " , \t" << neighborWeight[3] << endl;
}


void TaskSubgroup::UpdateLastPos() {
	//
	llastPos[0] = ltaskCenter[0];
	llastPos[1] = ltaskCenter[1];
	rlastPos[0] = rtaskCenter[0];
	rlastPos[1] = rtaskCenter[1];
}

vector<int> TaskSubgroup::TrialMove() {
	// update lastPositions
	for (int i = 0; i < ltaskNumber; ++i) {
		ltasks[i]->lastPosition.x = ltasks[i]->taskPoint.x;
		ltasks[i]->lastPosition.y = ltasks[i]->taskPoint.y;
	}
	for (int i = 0; i < rtaskNumber; ++i) {
		rtasks[i]->lastPosition.x = rtasks[i]->taskPoint.x;
		rtasks[i]->lastPosition.y = rtasks[i]->taskPoint.y;
	}

	// trial move of the overall body
	float oneStep = random();
	char move = ' ';
	vector<int> trial(2, 0); // x, y

	float interval1 = neighborWeight[0];
	float interval2 = interval1 + neighborWeight[1];
	float interval3 = interval2 + neighborWeight[2];
	float interval4 = interval3 + neighborWeight[3];
	if (interval4) { // if not all zero
		if (0 <= oneStep && oneStep < interval1) { trial[1] = 1; move = 'u'; }               // up
		else if (interval1 <= oneStep && oneStep < interval2) { trial[1] = -1; move = 'd'; } // down
		else if (interval2 <= oneStep && oneStep < interval3) { trial[0] = 1; move = 'l'; }  // left
		else if (interval3 <= oneStep && oneStep <= interval4) { trial[0] = -1; move = 'r'; } // right

		for (int i = 0; i < ltaskNumber; ++i) {
			ltasks[i]->tendPosition.x = ltasks[i]->taskPoint.x + trial[0];
			ltasks[i]->tendPosition.y = ltasks[i]->taskPoint.y + trial[1];
		}
		 
		for (int i = 0; i < rtaskNumber; ++i) {
			rtasks[i]->tendPosition.x = rtasks[i]->taskPoint.x + trial[0];
			rtasks[i]->tendPosition.y = rtasks[i]->taskPoint.y + trial[1];
		}

		for (int i = 0; i < 2; ++i) {
			ltendPos[i] = ltaskCenter[i] + trial[i];
			rtendPos[i] = rtaskCenter[i] + trial[i];
		}
	}
	
	cout << "Section of choice: (0, " << interval1 << "), (" << interval1 << ", " << interval2 <<
		"), (" << interval2 << ", " << interval3 << "), (" << interval3 << ", " << interval4 << ")" << endl;
	cout << "Trial move choice:  " << oneStep << ", " << move << endl;
	return trial;
}

void TaskSubgroup::RecoverTendPos() {
	for (int i = 0; i < ltaskNumber; ++i) {
		ltasks[i]->tendPosition.x = ltasks[i]->taskPoint.x;
		ltasks[i]->tendPosition.y = ltasks[i]->taskPoint.y;
	}
	for (int i = 0; i < rtaskNumber; ++i) {
		rtasks[i]->tendPosition.x = rtasks[i]->taskPoint.x;
		rtasks[i]->tendPosition.y = rtasks[i]->taskPoint.y;
	}
}

bool TaskSubgroup::OverallMove(MatrixMap* world, int direction) {
	UpdateWeights(world, direction); // update neighborWeight
	
	vector<int> trial = TrialMove(); // trial move // x, y  
	bool moveOK = MoveCheck(world, trial);
	if (moveOK) {
		moveOK = false;
		if (llastPos[0] != ltendPos[0] || llastPos[1] != ltendPos[1] ||
			rlastPos[0] != rtendPos[0] || rlastPos[1] != rtendPos[1]) moveOK = true;
	}
	UpdateLastPos();
	cout << endl << "Real move check: " << moveOK << endl;
	if (moveOK) { // check OK

		vector<int> collision(2);
		int step = Move(world, trial, trial, collision); // move
		moveStep += step;
		return true;
	}
	else {
		RecoverTendPos();
		return false; // 4 sides are obstacles
	}
}

int TaskSubgroup::Move(MatrixMap* world, vector<int> trial_left, vector<int> trial_right, vector<int> collision) {
	int step = 0; // count the move step, could be 0,1,2
	
	if (trial_left[0] == trial_right[0] && trial_left[1] == trial_right[1]) { // overall move
		cout << "Overall move:  left: \t" << trial_left[0] << " , " << trial_left[1] 
			<< "\tright\t" << trial_right[0] << " , " << trial_right[1] << endl;
		if (!collision[0] && !collision[1]) {
			// clear the map
			for (int i = 0; i < ltaskNumber; ++i)  // left
				world->map_task(ltasks[i]->taskPoint.x, ltasks[i]->taskPoint.y) = 0;
			for (int i = 0; i < rtaskNumber; ++i)  // right
				world->map_task(rtasks[i]->taskPoint.x, rtasks[i]->taskPoint.y) = 0;

			for (int i = 0; i < ltaskNumber; ++i) {   // left
				world->map_task(ltasks[i]->tendPosition.x, ltasks[i]->tendPosition.y) = ltasks[i]->id;
				ltasks[i]->taskPoint.x = ltasks[i]->taskPoint.x + trial_left[0];
				ltasks[i]->taskPoint.y = ltasks[i]->taskPoint.y + trial_left[1];
				ltasks[i]->step++;
			}
			// update the boundary
			lboundary[0] = lboundary[0] + trial_left[0];  // max X 
			lboundary[1] = lboundary[1] + trial_left[0];  // min X
			lboundary[2] = lboundary[2] + trial_left[1];  // max Y
			lboundary[3] = lboundary[3] + trial_left[1];  // min Y
			// update the center
			for (int i = 0; i < 2; ++i) ltaskCenter[i] += trial_left[i];
			
			for (int i = 0; i < rtaskNumber; ++i) {  // right
				world->map_task(rtasks[i]->tendPosition.x, rtasks[i]->tendPosition.y) = rtasks[i]->id;
				rtasks[i]->taskPoint.x = rtasks[i]->taskPoint.x + trial_right[0];
				rtasks[i]->taskPoint.y = rtasks[i]->taskPoint.y + trial_right[1];
				rtasks[i]->step++;
			}
			// update the boundary
			rboundary[0] = rboundary[0] + trial_right[0];  // max X 
			rboundary[1] = rboundary[1] + trial_right[0];  // min X
			rboundary[2] = rboundary[2] + trial_right[1];  // max Y
			rboundary[3] = rboundary[3] + trial_right[1];  // min Y
			// update the center
			for (int i = 0; i < 2; ++i) rtaskCenter[i] += trial_right[i];

			step += 1;
		} // otherwise not move
		else
			RecoverTendPos();
	}
	else { // separation move
		cout << "Separation move: left: \t" << trial_left[0] << " , " << trial_left[1] 
			<< "\tright\t" << trial_right[0] << " , " << trial_right[1] << endl;
		if (!collision[0]) { // left is clear
			for (int i = 0; i < ltaskNumber; ++i)   // clear the map
				world->map_task(ltasks[i]->taskPoint.x, ltasks[i]->taskPoint.y) = 0;
			for (int i = 0; i < ltaskNumber; ++i) {   // left
				world->map_task(ltasks[i]->tendPosition.x, ltasks[i]->tendPosition.y) = ltasks[i]->id;
				ltasks[i]->taskPoint.x = ltasks[i]->taskPoint.x + trial_left[0];
				ltasks[i]->taskPoint.y = ltasks[i]->taskPoint.y + trial_left[1];
				ltasks[i]->step++;
			}
			// update the boundary
			lboundary[0] = lboundary[0] + trial_left[0];  // max X 
			lboundary[1] = lboundary[1] + trial_left[0];  // min X
			lboundary[2] = lboundary[2] + trial_left[1];  // max Y
			lboundary[3] = lboundary[3] + trial_left[1];  // min Y
			// update the center
			for (int i = 0; i < 2; ++i) ltaskCenter[i] += trial_left[i];

			step += 1;
		}
		else {
			for (int i = 0; i < ltaskNumber; ++i) {
				ltasks[i]->tendPosition.x = ltasks[i]->taskPoint.x;
				ltasks[i]->tendPosition.y = ltasks[i]->taskPoint.y;
			}
		}
		if (!collision[1]) { // right is clear
			for (int i = 0; i < rtaskNumber; ++i)   // clear the map
				world->map_task(rtasks[i]->taskPoint.x, rtasks[i]->taskPoint.y) = 0;
			for (int i = 0; i < rtaskNumber; ++i) {  // right
				world->map_task(rtasks[i]->tendPosition.x, rtasks[i]->tendPosition.y) = rtasks[i]->id;
				rtasks[i]->taskPoint.x = rtasks[i]->taskPoint.x + trial_right[0];
				rtasks[i]->taskPoint.y = rtasks[i]->taskPoint.y + trial_right[1];
				rtasks[i]->step++;
			}
			// update the boundary
			rboundary[0] = rboundary[0] + trial_right[0];  // max X 
			rboundary[1] = rboundary[1] + trial_right[0];  // min X
			rboundary[2] = rboundary[2] + trial_right[1];  // max Y
			rboundary[3] = rboundary[3] + trial_right[1];  // min Y
			// update the center
			for (int i = 0; i < 2; ++i) rtaskCenter[i] += trial_right[i];

			step += 1;
		}
		else {
			for (int i = 0; i < rtaskNumber; ++i) {
				rtasks[i]->tendPosition.x = rtasks[i]->taskPoint.x;
				rtasks[i]->tendPosition.y = rtasks[i]->taskPoint.y;
			}
		}
	}
	// update centers
	for (int i = 0; i < 2; ++i)
		taskCenter[i] = float(ltaskCenter[i] + rtaskCenter[i]) / 2;

	return step;
}

bool TaskSubgroup::MoveCheck(MatrixMap* world, vector<int> trial){
	bool free = false; // left, right
	if (!PartMoveCheck(world, trial, 'l') && !PartMoveCheck(world, trial, 'r')) free = true;
	return free;
}

#pragma endregion

// Task

#pragma region Task

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
	bool ReadTask(string data_dir);
	bool GenerateTree();
	bool GenerateTree(MatrixMap* world);
	void Bisect(BinNode<vector<int>>*, BinNode<char>*);
	void Segment(BinNode<vector<int>>* node, BinNode<char>* segNode, MatrixMap* world);
	vector<int> GetMembers(BinNode<vector<int>>* node, vector<int> targetNode, int taskPoint, int countLayer, int targetLayer);
	vector<int> GetPeers(BinNode<vector<int>>* node, vector<int> peerNode, int taskPoint, int countLayer, int targetLayer);
	char GetSegDirection(BinNode<char>* segNode, BinNode<vector<int>>* assNode, char segDir, int taskPoint, int countLayer, int targetLayer);
	char GetChildSide(BinNode<vector<int>>* node, char childSide, int taskPoint, int countLayer, int targetLayer);

	vector<vector<int>> Comparison(vector<Point> left_component, vector<Point> right_component, MatrixMap* world);
	void PushAll(string);      // Store the currentTargets in allTargets or allExtendedPoints;
	void Display(string);      // display task positions of steps
	void Display(int);
	bool UpdateTaskmap(MatrixMap*, int);

private:
};

bool
Task::ReadTask(string data_dir) {
	ifstream f;                // Task.txt 中第一行为目标目标数目，后面每行为ID和坐标
	////read the final target points
	f.open(data_dir+"Task.txt", ifstream::in);
	if (f) {
		f >> taskNum;
		//TaskPoint tempTask;
		int task_id, task_x, task_y;
		int x_sum = 0;
		int y_sum = 0;
		char c;
		for (int i = 0; i < taskNum; i++) {
			f >> task_id >> c >> task_x >> c >> task_y;
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
		if (LogFlag)
			RecordLog("Read task: Failed to open the task file. Please check the filename.");
	}
	f.close();
	
	//// read the start points
	f.open(data_dir+"Robot_Init_Position.txt", ifstream::in);
	if (f) {
		f >> robotNum;
		if (robotNum != taskNum) {
			cout << "The robot NO. is not equal to task NO. Please check the robot NO.!" << endl;
			if (LogFlag)
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
		if (LogFlag)
			RecordLog("Read task: Failed to open the Robot_Init_Position file. Please check the filename.");
	}
	f.close();
	cout << "Finish to read the task and the robot initial position." << endl << endl;
	if (LogFlag)
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
			if (finalTargets[ids[j]]->taskPoint.x > finalTargets[ids[i]]->taskPoint.x)
				countX++;
			// split line is y = finalTargets[i].taskPoint.y
			if (finalTargets[ids[j]]->taskPoint.y > finalTargets[ids[i]]->taskPoint.y)
				countY++;
		}
		// comparison for the maximum
		double select = random();
		double decision = 0.5;
		if (select > decision) {
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
		else {
			if (countY * (compSize - countY) > max) {  // y direction
				max = countY * (compSize - countY);
				line = ids[i];
				xory = 'y';
			}
			if (countX * (compSize - countX) > max) {  // x direction
				max = countX * (compSize - countX);
				line = ids[i];
				xory = 'x';
			}
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
	node->lChild->parent = node;
	node->rChild->parent = node;

	segNode->data = xory;
	segNode->lChild = new BinNode<char>('a'); // a means nonsense
	segNode->rChild = new BinNode<char>('a');
	segNode->lChild->parent = segNode;
	segNode->rChild->parent = segNode;

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

void
Task::Segment(BinNode<vector<int>>* node, BinNode<char>* segNode, MatrixMap* world) {
	if (!node) return;
	vector<int> components = node->data;  // components (robot group)是 node中包含的targets的ID
	int compSize = components.size();
	if (compSize <= 1) return;  // components 元素仅一个时，停止分割
	vector<int> ids;   // 找到ID对应的targets在finalTargets中的位置，存入ids中
	for (int i = 0; i < compSize; i++)
		for (int j = 0; j < finalTargets.size(); j++)
			if (finalTargets[j]->id == components[i])    // ids[i] <-> components[i]
				ids.push_back(j);
	
	// initialization
	int max = INT_MIN;
	int line = 0;
	char xory = ' ';
	vector<int> left;
	vector<int> right;
	for (int i = 0; i < compSize; i++) {   // every possible split line 
		for (int j = 0; j < compSize; j++) {
			double select = random();
			double decision = 0.5;
			if (select > decision) {
				// split line is x = finalTargets[i].taskPoint.x
				{
					vector<Point> left_component, right_component;
					if (finalTargets[ids[j]]->taskPoint.x > finalTargets[ids[i]]->taskPoint.x) {
						left_component.push_back(finalTargets[ids[j]]->taskPoint);
					}
					else {
						right_component.push_back(finalTargets[ids[j]]->taskPoint);
					}

					// obtain the two parts
					vector<vector<int>> new_component_id = Comparison(left_component, right_component, world);
					vector<int> left_id = new_component_id[0];
					vector<int> right_id = new_component_id[1];

					// comparison for the maximum
					if (left_id.size() * (compSize - left_id.size()) > max) {  // x direction
						max = left_id.size() * (compSize - left_id.size());
						xory = 'x';
						left.swap(vector<int>());
						right.swap(vector<int>());
						left = left_id;
						right = right_id;
					}
				}
				// split line is y = finalTargets[i].taskPoint.y
				{
					vector<Point> left_component, right_component;
					if (finalTargets[ids[j]]->taskPoint.y > finalTargets[ids[i]]->taskPoint.y) {
						left_component.push_back(finalTargets[ids[j]]->taskPoint);
					}
					else {
						right_component.push_back(finalTargets[ids[j]]->taskPoint);
					}

					// obtain the two parts
					vector<vector<int>> new_component_id = Comparison(left_component, right_component, world);
					vector<int> left_id = new_component_id[0];
					vector<int> right_id = new_component_id[1];

					// comparison for the maximum
					if (left_id.size() * (compSize - left_id.size()) > max) {  // x direction
						max = left_id.size() * (compSize - left_id.size());
						xory = 'y';
						left.swap(vector<int>());
						right.swap(vector<int>());
						left = left_id;
						right = right_id;
					}
				}
			}
			else {
				// split line is y = finalTargets[i].taskPoint.y
				{
					vector<Point> left_component, right_component;
					if (finalTargets[ids[j]]->taskPoint.y > finalTargets[ids[i]]->taskPoint.y) {
						left_component.push_back(finalTargets[ids[j]]->taskPoint);
					}
					else {
						right_component.push_back(finalTargets[ids[j]]->taskPoint);
					}

					// obtain the two parts
					vector<vector<int>> new_component_id = Comparison(left_component, right_component, world);
					vector<int> left_id = new_component_id[0];
					vector<int> right_id = new_component_id[1];

					// comparison for the maximum
					if (left_id.size() * (compSize - left_id.size()) > max) {  // x direction
						max = left_id.size() * (compSize - left_id.size());
						xory = 'y';
						left.swap(vector<int>());
						right.swap(vector<int>());
						left = left_id;
						right = right_id;
					}
				}
				// split line is x = finalTargets[i].taskPoint.x
				{
					vector<Point> left_component, right_component;
					if (finalTargets[ids[j]]->taskPoint.x > finalTargets[ids[i]]->taskPoint.x) {
						left_component.push_back(finalTargets[ids[j]]->taskPoint);
					}
					else {
						right_component.push_back(finalTargets[ids[j]]->taskPoint);
					}

					// obtain the two parts
					vector<vector<int>> new_component_id = Comparison(left_component, right_component, world);
					vector<int> left_id = new_component_id[0];
					vector<int> right_id = new_component_id[1];

					// comparison for the maximum
					if (left_id.size() * (compSize - left_id.size()) > max) {  // x direction
						max = left_id.size() * (compSize - left_id.size());
						xory = 'x';
						left.swap(vector<int>());
						right.swap(vector<int>());
						left = left_id;
						right = right_id;
					}
				}
			}
		}
	}

	// call self
	node->lChild = new BinNode<vector<int>>(left);
	node->rChild = new BinNode<vector<int>>(right);

	segNode->data = xory;
	segNode->lChild = new BinNode<char>('a'); // a means nonsense
	segNode->rChild = new BinNode<char>('a');

	
	Segment(node->lChild, segNode->lChild, world);
	Segment(node->rChild, segNode->rChild, world);
}


// manipulate the assembly tree
// according to one task point, find the task group in a layer
vector<int>
Task::GetMembers(BinNode<vector<int>>* node, vector<int> targetNode, int taskPoint, int countLayer, int targetLayer) {
	// return
	if (node == NULL)
		return targetNode;

	// get data
	if (targetLayer >= 0 && countLayer == targetLayer) {
		for (int i = 0; i < node->data.size(); i++) {
			if (node->data[i] == taskPoint) {
				targetNode = node->data;
				return targetNode;
			}
		}
	}

	// recursion
	targetNode = GetMembers(node->lChild, targetNode, taskPoint, countLayer +1, targetLayer);
	targetNode = GetMembers(node->rChild, targetNode, taskPoint, countLayer +1, targetLayer);

	return targetNode;
}

// according to one task point, find the peer task group in a layer
vector<int>
Task::GetPeers(BinNode<vector<int>>* node, vector<int> peerNode, int taskPoint, int countLayer, int targetLayer) {
	// return
	if (node == NULL)
		return peerNode;

	// get data
	if (targetLayer >= 0 && countLayer == targetLayer) {
		vector<int> ldata = node->parent->lChild->data;
		vector<int> rdata = node->parent->rChild->data;
		
		for (int i = 0; i < node->data.size(); i++) {
			if (node->data[i] == taskPoint) {
				if (node->data == ldata) { // if left child
					peerNode = rdata;
				}
				else if (node->data == rdata) { // if right child
					peerNode = ldata;
				}
				return peerNode;
			}
		}
	}

	// recursion
	peerNode = GetPeers(node->lChild, peerNode, taskPoint, countLayer + 1, targetLayer);
	peerNode = GetPeers(node->rChild, peerNode, taskPoint, countLayer + 1, targetLayer);

	return peerNode;
}

// find the child side from the assembly tree
char
Task::GetChildSide(BinNode<vector<int>>* node, char childSide, int taskPoint, int countLayer, int targetLayer) {
	// return
	if (node == NULL)
		return childSide;

	// get data
	if (targetLayer >= 0 && countLayer == targetLayer) {
		vector<int> ldata = node->parent->lChild->data;
		vector<int> rdata = node->parent->rChild->data;

		for (int i = 0; i < node->data.size(); i++) {
			if (node->data[i] == taskPoint) {
				if (node->data == ldata) { // if left child
					return 'l';
				}
				else if (node->data == rdata) { // if right child
					return 'r';
				}
			}
		}
	}

	// recursion
	childSide = GetChildSide(node->lChild, childSide, taskPoint, countLayer + 1, targetLayer);
	childSide = GetChildSide(node->rChild, childSide, taskPoint, countLayer + 1, targetLayer);

	return childSide;
}

// find the segmentation direction from the segTree
char 
Task::GetSegDirection(BinNode<char>* segNode, BinNode<vector<int>>* assNode, char segDir, int taskPoint, int countLayer, int targetLayer) {
	// return the segmentation direction between the group and its peer group
	if (assNode == NULL || segNode == NULL)
		return segDir;

	// get data
	if (targetLayer >= 0 && countLayer == targetLayer) {
		for (int i = 0; i < assNode->data.size(); i++) {
			if (assNode->data[i] == taskPoint) {
				segDir = segNode->parent->data;
				return segDir;
			}
		}
	}

	// recursion
	segDir = GetSegDirection(segNode->lChild, assNode->lChild, segDir, taskPoint, countLayer + 1, targetLayer);
	segDir = GetSegDirection(segNode->rChild, assNode->rChild, segDir, taskPoint, countLayer + 1, targetLayer);

	return segDir;
}

// input two components, output two maximum parts
vector<vector<int>> 
Task::Comparison(vector<Point> left_component, vector<Point> right_component, MatrixMap* world) {
	vector<int> left;
	vector<int> right;

	int compSize = left_component.size() + right_component.size();
	if (world->IsOneGroup(left_component) && world->IsOneGroup(right_component)) { // properly two parts
		for (int i = 0; i < left_component.size(); ++i) {
			left.push_back(world->map_task(left_component[i].x, left_component[i].y));
		}
		for (int i = 0; i < right_component.size(); ++i) {
			right.push_back(world->map_task(right_component[i].x, right_component[i].y));
		}
	}
	else { // there are more than two parts
		int max = INT_MIN;
		// left_components
		{
			vector<vector<Point>> main_comps = world->Clustering(left_component);
			for (int i = 0; i < main_comps.size(); ++i) {
				vector<Point> first_comps = main_comps[i];
				// form the second parts
				vector<Point> second_comps = right_component;
				for (int j = 0; j < main_comps.size(); ++j) {
					if (j == i)
						continue;
					else
						second_comps.insert(second_comps.end(), main_comps[j].begin(), main_comps[j].end());
				}

				// compare
				if (world->IsOneGroup(second_comps)) { // second_comps is one part
					if (first_comps.size() * (compSize - first_comps.size()) > max) {
						max = first_comps.size() * (compSize - first_comps.size());
						// push to the result
						left.swap(vector<int>());
						right.swap(vector<int>());
						for (int k = 0; k < first_comps.size(); ++k) {
							left.push_back(world->map_task(first_comps[k].x, first_comps[k].y));
						}
						for (int k = 0; k < second_comps.size(); ++k) {
							right.push_back(world->map_task(second_comps[k].x, second_comps[k].y));
						}
					}
				}
			}
		}
		// right_components
		{
			vector<vector<Point>> main_comps = world->Clustering(right_component);
			for (int i = 0; i < main_comps.size(); ++i) {
				vector<Point> first_comps = main_comps[i];
				// form the second parts
				vector<Point> second_comps = left_component;
				for (int j = 0; j < main_comps.size(); ++j) {
					if (j == i)
						continue;
					else
						second_comps.insert(second_comps.end(), main_comps[j].begin(), main_comps[j].end());
				}

				// compare
				if (world->IsOneGroup(second_comps)) { // second_comps is one part
					if (first_comps.size() * (compSize - first_comps.size()) > max) {
						max = first_comps.size() * (compSize - first_comps.size());
						// push to the result
						left.swap(vector<int>());
						right.swap(vector<int>());
						for (int k = 0; k < first_comps.size(); ++k) {
							left.push_back(world->map_task(first_comps[k].x, first_comps[k].y));
						}
						for (int k = 0; k < second_comps.size(); ++k) {
							right.push_back(world->map_task(second_comps[k].x, second_comps[k].y));
						}
					}
				}
			}
		}
	}

	vector<vector<int>> left_right;
	left_right.push_back(left);
	left_right.push_back(right);
	return left_right;
}

bool
Task::GenerateTree() {
	// Initially, all ids into root
	vector<int> ids;
	for (int i = 0; i < taskNum; i++) ids.push_back(finalTargets[i]->id);
	if (ids.size() == 0) {
		cout << "Generate tree: Task NO. is 0. Failed to generate the assembly tree!" << endl;
		if (LogFlag)
			RecordLog("Generate tree: Task NO. is 0. Failed to generate the assembly tree!");
		return false;
	}
	else {
		BinNode<vector<int>>* root = AssemblyTree.insertASRoot(ids);
		BinNode<char>* segRoot = SegTree.insertASRoot('x');
		Bisect(root, segRoot);
		AssemblyTree.DisplayTree();
		//cout << "Flattern Tree:" << endl;
		//SegTree.display(SegTree.root());
		cout << endl;
		if (LogFlag)
			RecordLog("Success to generate the assembly tree!");
		return true;
	}
}

bool
Task::GenerateTree(MatrixMap* world) {
	// Initially, all ids into root
	vector<int> ids;
	for (int i = 0; i < taskNum; i++) ids.push_back(finalTargets[i]->id);
	if (ids.size() == 0) {
		cout << "Generate tree: Task NO. is 0. Failed to generate the assembly tree!" << endl;
		if (LogFlag)
			RecordLog("Generate tree: Task NO. is 0. Failed to generate the assembly tree!");
		return false;
	}
	else {
		BinNode<vector<int>>* root = AssemblyTree.insertASRoot(ids);
		BinNode<char>* segRoot = SegTree.insertASRoot('x');
		Segment(root, segRoot, world);
		AssemblyTree.DisplayTree();
		//cout << "Flattern Tree:" << endl;
		//SegTree.display(SegTree.root());
		cout << endl;
		if (LogFlag)
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

#pragma endregion
