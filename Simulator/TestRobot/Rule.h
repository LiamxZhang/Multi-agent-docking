#pragma once
// Rule定义路径规划，机器人移动等

#include <iostream>
#include <vector>

#include "Point.h"
#include "Log.h"

using namespace std;

class Rule {
private:
	int mapRowNumber;                      // the number of map row
	int mapColumnNumber;                   // the number of map column
	int robotNumber;                       // the number of robot
	int taskNumber;                           // the number of task
	int globalTime;                        // the time of system


	int taskId;                            // taskId is used to record the id of task
	vector<float> bidTable;                // the talbe of all robot's bid price
	vector<Point> robotInitPosition;       // the init position of robot
	vector<Point> robotCurrentPosition;    // the current position of robot
	vector<Point> robotTargetPosition;     // the target position of robot
	vector<Task> ToDoTask, DoingTask, DoneTask; // three vector of task
	vector<Task> waitToAssigned;             // wait to be assigned
	//vector<vector<WeightPoint>> weightMap;   // the weight of map
public:
	vector<vector<int> > map;               // use two dimension to record the map
	//changed by jiaming, it supposed to be private

	Rule() :mapRowNumber(0), mapColumnNumber(0), globalTime(0), robotNumber(0), taskNumber(0),
		taskId(0) {
		cout << "Construct a Object of Class Rule " << endl; RecordLog("Construct a Object of Class Rule ");
	}
	inline int getMapRowNumber() { return mapRowNumber; }
	inline int getMapColumnNumber() { return mapColumnNumber; }
	inline int getRobotNumber() { return robotNumber; }
	inline bool setGlobalTime(int newGlobalTime) { globalTime = newGlobalTime; return true; }
	inline int getGlobalTime() { return globalTime; }
	//vector<vector<int> > getMap() { return AvoidMap; }
	//vector<vector<WeightPoint> > getWeightMap() { return weightMap; }
	Point getRobotInitPosition(int RobotNumber) { return robotInitPosition[RobotNumber]; }
	bool setRobotCurrentPosition(int RobotNumber, Point Position) {
		robotCurrentPosition[RobotNumber] = Position;
		robotCurrentPositionCopy[RobotNumber] = Position;
		return true;
	}
	bool setRobotTargetPosition(int RobotNumber, Point Position) {
		robotTargetPosition[RobotNumber] = Position;
		return true;
	}
	bool Dock();
	bool ReadMap();
	bool GlobalTime();
	bool ReadRobotInitPosition();
	bool RecordCurrentAndTargetPosition();
	bool GenerateTask();
	bool ReadTask();

	//CutResult Cut(vector<Task> InputVector);
	//CutResult MoveRow(CutResult InputVector);
	//CutResult MoveVector(CutResult InputVector);
	int PrintVector(vector<Task> InputVector);
	int ShortDistance(Point& source, Point& target);
	int ShortPath(Point& source, Point& target);
	bool AssignTask();
	friend class Robot;
	friend int main();
};

