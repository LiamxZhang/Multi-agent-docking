#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <ctime>
#include <math.h>
#include <algorithm>
#include <queue>
#include <map>
#include <set>
#include <windows.h>

#include "Point.h"
#include "Task.h"
#include "Map.h"
#include "Robot.h"
#include "CommonFunctions.h"
#include "Log.h"

#define WaitTime 500

using namespace std;

class WaveAlg {
public:
	// main function
	void Processing();

	// supporting functions
	void TaskExtension(Task* task, MatrixMap* map);
	void WavePropagation(vector<vector<int>> candidateGroup, vector<int> allstuckGroups, vector<TaskSubgroup>* taskGroups, MatrixMap* map);
	vector<int> Separation(vector<TaskSubgroup>* taskGroups, Task* task, MatrixMap* map);
	vector<vector<int>> FindEndPoint(vector<int> incompleteGroup, vector<TaskSubgroup>* taskGroups, MatrixMap* map);
	vector<vector<int>> FindNeighbors(vector<int> candidateGroup, vector<int> allstuckGroups, vector<TaskSubgroup>* taskGroups, MatrixMap* map);
	vector<vector<int>> PartNeighbors(vector<int> boundary, vector<int> allstuckGroups, vector<TaskSubgroup>* taskGroups, MatrixMap* map);
	int WhichGroup(int pointID, vector<TaskSubgroup>* taskGroups);


private:
};


void WaveAlg::Processing() {
	// read map, the origin is in the leftmost top,  x means rows, y means columns
	MatrixMap* world = new MatrixMap();
	world->ReadMap();
	world->Display("obstacle"); //world.Display();

	// read task, generate assembly tree
	Task* task = new Task();
	task->ReadTask();
	task->GenerateTree();
	//cout << endl << "Depth: " << task->AssemblyTree.depth(task->AssemblyTree.root()) << endl;
	//cout << endl << world->TaskCheck(1, task->AssemblyTree.leaves()[0]->data, 2) << endl; 

	// create the robots
	vector<Robot*> robot;
	for (int i = 0; i < task->robotNum; i++) {
		Robot* temp = new Robot(task->robotNum, task->startPoints[i]->id, task->startPoints[i]->taskPoint, world->RowNum, world->ColNum);
		robot.push_back(temp);
	}

	// Extend the task components, according to the assembly tree
	TaskExtension(task, world);

	// assign the task to the closest robots using optimization (or bid)
	AssignTaskToRobot(task, robot);
	// show the task extension process
	RecordTaskExtend(task, robot);

	// ID to index
	vector<vector<int>> idToIndex = IDtoIndex(robot);
	vector<int> tID2index = idToIndex[0];  // input: task ID  output: robot index
	//vector<int> rID2index = idToIndex[1];  // input: robot ID output: robot index

	// Robot movement
	RobotMove(task, robot, world, tID2index);
	
	//system("pause");
	Recover(task);
	return;
}

void WaveAlg::TaskExtension(Task* task, MatrixMap* map) {
	task->PushAll("allExtendedPoints");

	int depth = task->AssemblyTree.depth(task->AssemblyTree.root());

	for (int i = 0; i < depth - 1; i++) {
		// construct the task subgroups
		vector<TaskSubgroup>* taskGroups = new vector<TaskSubgroup>();
		GetTaskSubgroups(taskGroups, task->AssemblyTree.root(), task->SegTree.root(), task, 0, i);

		bool sepComplete = false;
		while (!sepComplete) {
			// try separation
			vector<int> incompleteGroup = Separation(taskGroups, task, map);
			task->PushAll("allExtendedPoints");

			// EndCheck: if all Done, complete; otherwise, move
			sepComplete = true;
			for (int j = 0; j < taskGroups->size(); ++j) {
				if (!(*taskGroups)[j].sepDone) { sepComplete = false; break; }
			}

			// find the two end
			vector<vector<int>> endGroups = FindEndPoint(incompleteGroup, taskGroups, map);
			cout << "endGroups[0]'s size is :   " << endGroups[0].size() << endl;

			cout << "End groups are:   ";
			for (int k = 0; k < endGroups[0].size(); ++k) {
				cout << endGroups[0][k] << " (" << (*taskGroups)[endGroups[0][k]].leader << "),\t";
			}
			cout << endl;

			// recursion
			WavePropagation(endGroups, endGroups[0], taskGroups, map);
			task->PushAll("allExtendedPoints");

			map->Display("task");
		}
		task->PushAll("allTargets");
	}
	// task->PushAll("allTargets");
}

// separation operation
// return sep incomplete groups
vector<int> WaveAlg::Separation(vector<TaskSubgroup>* taskGroups, Task* task, MatrixMap* map) {
	vector<int> incompete;
	// each group separation
	for (int i = 0; i < taskGroups->size(); ++i) {
		if (!(*taskGroups)[i].sepDone) {
			(*taskGroups)[i].Separation(map); // normal separation // return the endpoints
			if (!(*taskGroups)[i].sepDone)
				incompete.push_back(i);
		}
	}
	return incompete;
}

// find the two end
// return the nearest task groups in the two ends
vector<vector<int>> WaveAlg::FindEndPoint(vector<int> incompleteGroup, vector<TaskSubgroup>* taskGroups, MatrixMap* map) {
	int range = (*taskGroups)[0].range;
	vector<int> endGroups;
	vector<int> directions; // 0 no direction, 1 up, 2 down, 3 left, 4 right
	int up = 1;
	int down = 2;
	int left = 3;
	int right = 4;

	for (int i = 0; i < incompleteGroup.size(); ++i) {
		// direction
		char direction = (*taskGroups)[incompleteGroup[i]].segDir;
		// vector<int> lboundary; // max X, min X, max Y, min Y
		// vector<int> rboundary; // max X, min X, max Y, min Y
		if (direction == 'x') {
			{// left
				vector<int> boundary = (*taskGroups)[incompleteGroup[i]].lboundary; // max X, min X, max Y, min Y
				int minX, maxX, minY, maxY;
				boundary[0] + range + 1 < map->ColNum - 1 ? maxX = boundary[0] + range + 1 : maxX = map->ColNum - 1; // max X
				boundary[1] - range > 0 ? minX = boundary[1] - range : minX = 0;                                 // min X
				boundary[2] + range < map->RowNum - 1 ? maxY = boundary[2] + range : maxY = map->RowNum - 1; // max Y
				boundary[3] - range > 0 ? minY = boundary[3] - range : minY = 0;                                 // min Y

				for (int y = minY; y <= maxY; ++y) {
					for (int x = boundary[0]; x <= maxX; ++x) {
						if (map->map_obstacle(x, y)) break;
						if (map->map_task(x, y)) { // find the task points
							// find which group it belongs to
							int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
							// if not include, include
							vector<int>::iterator iter = find(endGroups.begin(), endGroups.end(), new_groupID);
							if (iter == endGroups.end()) {
								endGroups.push_back(new_groupID);
								directions.push_back(left);
							}
							break;
						}
					}
				}
			}
			{// right
				vector<int> boundary = (*taskGroups)[incompleteGroup[i]].rboundary; // max X, min X, max Y, min Y
				int minX, maxX, minY, maxY;
				boundary[0] + range < map->ColNum - 1 ? maxX = boundary[0] + range : maxX = map->ColNum - 1; // max X
				boundary[1] - range - 1 > 0 ? minX = boundary[1] - range - 1 : minX = 0;                                 // min X
				boundary[2] + range < map->RowNum - 1 ? maxY = boundary[2] + range : maxY = map->RowNum - 1; // max Y
				boundary[3] - range > 0 ? minY = boundary[3] - range : minY = 0;                                 // min Y

				for (int y = minY; y <= maxY; ++y) {
					for (int x = boundary[1]; x >= minX; --x) {
						if (map->map_obstacle(x, y)) break;
						if (map->map_task(x, y)) { // find the task points
							// find which group it belongs to
							int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
							// if not include, include
							vector<int>::iterator iter = find(endGroups.begin(), endGroups.end(), new_groupID);
							if (iter == endGroups.end()) {
								endGroups.push_back(new_groupID);
								directions.push_back(right);
							}
							break;
						}
					}
				}
			}
		}
		else if (direction == 'y') {
			{// left
				vector<int> boundary = (*taskGroups)[incompleteGroup[i]].lboundary; // max X, min X, max Y, min Y
				int minX, maxX, minY, maxY;
				boundary[0] + range < map->ColNum - 1 ? maxX = boundary[0] + range : maxX = map->ColNum - 1; // max X
				boundary[1] - range > 0 ? minX = boundary[1] - range : minX = 0;                                 // min X
				boundary[2] + range + 1 < map->RowNum - 1 ? maxY = boundary[2] + range + 1 : maxY = map->RowNum - 1; // max Y
				boundary[3] - range > 0 ? minY = boundary[3] - range : minY = 0;                                 // min Y

				for (int x = minX; x <= maxX; ++x) {
					for (int y = boundary[2]; y <= maxY; ++y) {
						if (map->map_obstacle(x, y)) break;
						if (map->map_task(x, y)) { // find the task points
							// find which group it belongs to
							int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
							// if not include, include
							vector<int>::iterator iter = find(endGroups.begin(), endGroups.end(), new_groupID);
							if (iter == endGroups.end()) {
								endGroups.push_back(new_groupID);
								directions.push_back(up);
							}
							break;
						}
					}
				}
			}
			{// right
				vector<int> boundary = (*taskGroups)[incompleteGroup[i]].rboundary; // max X, min X, max Y, min Y
				int minX, maxX, minY, maxY;
				boundary[0] + range < map->ColNum - 1 ? maxX = boundary[0] + range : maxX = map->ColNum - 1; // max X
				boundary[1] - range > 0 ? minX = boundary[1] - range : minX = 0;                                 // min X
				boundary[2] + range < map->RowNum - 1 ? maxY = boundary[2] + range : maxY = map->RowNum - 1; // max Y
				boundary[3] - range - 1 > 0 ? minY = boundary[3] - range - 1 : minY = 0;                                 // min Y

				for (int x = minX; x <= maxX; ++x) {
					for (int y = boundary[3]; y >= minY; --y) {
						if (map->map_obstacle(x, y)) break;
						if (map->map_task(x, y)) { // find the task points
							// find which group it belongs to
							int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
							// if not include, include
							vector<int>::iterator iter = find(endGroups.begin(), endGroups.end(), new_groupID);
							if (iter == endGroups.end()) {
								endGroups.push_back(new_groupID);
								directions.push_back(down);
							}
							break;
						}
					}
				}
			}
		}
	}

	vector<vector<int>> end;
	end.push_back(endGroups);
	end.push_back(directions);

	return end;
}

// find the around pairs, assume these are 
// return the nearest groups in 4 sides, and the directions
vector<vector<int>> WaveAlg::FindNeighbors(vector<int> candidateGroup, vector<int> allstuckGroups, vector<TaskSubgroup>* taskGroups, MatrixMap* map) {
	// candidateGroup: the current groups to be decide to move or to propagate; 
	// allstuckGroups: all stuck groups
	int range = (*taskGroups)[0].range;
	vector<int> new_neighbors;
	vector<int> directions; // 0 no direction, 1 up, 2 down, 3 left, 4 right
	for (int i = 0; i < candidateGroup.size(); ++i) {
		// test if it can move
		vector<int> trial(2);
		trial[1] = 1; // up
		bool up = (*taskGroups)[candidateGroup[i]].MoveCheck(map, trial);
		trial[1] = -1; // down
		bool down = (*taskGroups)[candidateGroup[i]].MoveCheck(map, trial);
		trial[1] = 0;
		trial[0] = 1; // left
		bool left = (*taskGroups)[candidateGroup[i]].MoveCheck(map, trial);
		trial[0] = -1; // right
		bool right = (*taskGroups)[candidateGroup[i]].MoveCheck(map, trial);
		/*
		if (up || down || left || right) {
			cout << "Group " << candidateGroup[i] << " can move. " << endl;
			continue; // at least one direction can move
		}
		*/

		vector<vector<int>> new_neighbors_l = PartNeighbors((*taskGroups)[candidateGroup[i]].lboundary, allstuckGroups, taskGroups, map);
		// neighbors IDs and the directions
		cout << "FindNeighbors: \t left:\t ";
		if (new_neighbors_l.size()) {
			for (int k = 0; k < new_neighbors_l[0].size(); ++k) {
				new_neighbors.push_back(new_neighbors_l[0][k]);
				directions.push_back(new_neighbors_l[1][k]);
				allstuckGroups.push_back(new_neighbors_l[0][k]);
				cout << new_neighbors_l[0][k] << " (" << (*taskGroups)[new_neighbors_l[0][k]].leader << "),\t";
			}
		}
		cout << endl;

		vector<vector<int>> new_neighbors_r = PartNeighbors((*taskGroups)[candidateGroup[i]].rboundary, allstuckGroups, taskGroups, map);
		cout << "\t right: \t";
		if (new_neighbors_r.size()) {
			for (int k = 0; k < new_neighbors_r[0].size(); ++k) {
				new_neighbors.push_back(new_neighbors_r[0][k]);
				directions.push_back(new_neighbors_r[1][k]);
				allstuckGroups.push_back(new_neighbors_r[0][k]);
				cout << new_neighbors_r[0][k] << " (" << (*taskGroups)[new_neighbors_r[0][k]].leader << "),\t";
			}
		}
		cout << endl;

		/*
		{// up
			vector<int> boundary = (*taskGroups)[candidateGroup[i]].lboundary; // max X, min X, max Y, min Y
			int minX, maxX, minY, maxY;
			boundary[0] + range < map->ColNum - 1 ? maxX = boundary[0] + range : maxX = map->ColNum - 1; // max X
			boundary[1] - range > 0 ? minX = boundary[1] - range : minX = 0;                                 // min X
			boundary[2] + range + 1 < map->RowNum - 1 ? maxY = boundary[2] + range + 1 : maxY = map->RowNum - 1; // max Y
			boundary[3] - range > 0 ? minY = boundary[3] - range : minY = 0;                                 // min Y

			for (int x = minX; x <= maxX; ++x) {
				for (int y = boundary[2]; y <= maxY; ++y) {
					if (map->map_task(x, y)) { // find the task points
						// find which group it belongs to
						int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
						// if never include
						vector<int>::iterator iter = find(allstuckGroups.begin(), allstuckGroups.end(), new_groupID);
						if (iter == allstuckGroups.end())
							neighbors.push_back(new_groupID);
						break;
					}
				}
			}
		}

		{// down
			vector<int> boundary = (*taskGroups)[candidateGroup[i]].rboundary; // max X, min X, max Y, min Y
			int minX, maxX, minY, maxY;
			boundary[0] + range < map->ColNum - 1 ? maxX = boundary[0] + range : maxX = map->ColNum - 1; // max X
			boundary[1] - range > 0 ? minX = boundary[1] - range : minX = 0;                                 // min X
			boundary[2] + range < map->RowNum - 1 ? maxY = boundary[2] + range : maxY = map->RowNum - 1; // max Y
			boundary[3] - range - 1 > 0 ? minY = boundary[3] - range - 1 : minY = 0;                                 // min Y

			for (int x = minX; x <= maxX; ++x) {
				for (int y = boundary[3]; y >= minY; --y) {
					if (map->map_task(x, y)) { // find the task points
						// find which group it belongs to
						int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
						// if not include, include
						vector<int>::iterator iter = find(allstuckGroups.begin(), allstuckGroups.end(), new_groupID);
						if (iter == allstuckGroups.end())
							neighbors.push_back(new_groupID);
						continue;
					}
				}
			}
		}

		{// left
			vector<int> boundary = (*taskGroups)[candidateGroup[i]].lboundary; // max X, min X, max Y, min Y
			int minX, maxX, minY, maxY;
			boundary[0] + range + 1 < map->ColNum - 1 ? maxX = boundary[0] + range + 1 : maxX = map->ColNum - 1; // max X
			boundary[1] - range > 0 ? minX = boundary[1] - range : minX = 0;                                 // min X
			boundary[2] + range < map->RowNum - 1 ? maxY = boundary[2] + range : maxY = map->RowNum - 1; // max Y
			boundary[3] - range > 0 ? minY = boundary[3] - range : minY = 0;                                 // min Y

			for (int y = minY; y <= maxY; ++y) {
				for (int x = boundary[0]; x <= maxX; ++x) {
					if (map->map_task(x, y)) { // find the task points
						// find which group it belongs to
						int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
						// if not include, include
						vector<int>::iterator iter = find(allstuckGroups.begin(), allstuckGroups.end(), new_groupID);
						if (iter == allstuckGroups.end())
							neighbors.push_back(new_groupID);
						continue;
					}
				}
			}
		}

		{// right
			vector<int> boundary = (*taskGroups)[candidateGroup[i]].rboundary; // max X, min X, max Y, min Y
			int minX, maxX, minY, maxY;
			boundary[0] + range < map->ColNum - 1 ? maxX = boundary[0] + range : maxX = map->ColNum - 1; // max X
			boundary[1] - range - 1 > 0 ? minX = boundary[1] - range - 1 : minX = 0;                                 // min X
			boundary[2] + range < map->RowNum - 1 ? maxY = boundary[2] + range : maxY = map->RowNum - 1; // max Y
			boundary[3] - range > 0 ? minY = boundary[3] - range : minY = 0;                                 // min Y

			for (int y = minY; y <= maxY; ++y) {
				for (int x = boundary[1]; x >= minX; --x) {
					if (map->map_task(x, y)) { // find the task points
						// find which group it belongs to
						int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
						// if not include, include
						vector<int>::iterator iter = find(allstuckGroups.begin(), allstuckGroups.end(), new_groupID);
						if (iter == allstuckGroups.end())
							neighbors.push_back(new_groupID);
						continue;
					}
				}
			}
		}
		*/

	}
	vector<vector<int>> neighbors;
	neighbors.push_back(new_neighbors);
	neighbors.push_back(directions);

	return neighbors;
}

vector<vector<int>> WaveAlg::PartNeighbors(vector<int> boundary, vector<int> allstuckGroups, vector<TaskSubgroup>* taskGroups, MatrixMap* map) {
	int range = (*taskGroups)[0].range;
	vector<int> new_neighbors;
	vector<int> directions; // 0 no direction, 1 up, 2 down, 3 left, 4 right
	int up = 1;
	int down = 2;
	int left = 3;
	int right = 4;
	int minX, maxX, minY, maxY;

	boundary[0] + range < map->ColNum - 1 ? maxX = boundary[0] + range : maxX = map->ColNum - 1;     // max X
	boundary[1] - range > 0 ? minX = boundary[1] - range : minX = 0;                                 // min X
	boundary[2] + range + 1 < map->RowNum - 1 ? maxY = boundary[2] + range + 1 : maxY = map->RowNum - 1; // max Y
	boundary[3] - range - 1 > 0 ? minY = boundary[3] - range - 1 : minY = 0;                             // min Y

	// up
	for (int x = minX; x <= maxX; ++x) {
		for (int y = boundary[2]; y <= maxY; ++y) {
			//if (map->map_obstacle(x, y)) break;
			if (map->map_task(x, y)) { // find the task points
				// find which group it belongs to
				int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
				// if never include
				vector<int>::iterator iter = find(allstuckGroups.begin(), allstuckGroups.end(), new_groupID);
				if (iter == allstuckGroups.end()) {
					new_neighbors.push_back(new_groupID);
					directions.push_back(up);
					allstuckGroups.push_back(new_groupID);
				}
				//break;
			}
		}
	}
	// down	
	for (int x = minX; x <= maxX; ++x) {
		for (int y = boundary[3]; y >= minY; --y) {
			//if (map->map_obstacle(x, y)) break;
			if (map->map_task(x, y)) { // find the task points
				// find which group it belongs to
				int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
				// if not include, include
				vector<int>::iterator iter = find(allstuckGroups.begin(), allstuckGroups.end(), new_groupID);
				if (iter == allstuckGroups.end()) {
					new_neighbors.push_back(new_groupID);
					directions.push_back(down);
					allstuckGroups.push_back(new_groupID);
				}
				//break;
			}
		}
	}

	boundary[0] + range + 1 < map->ColNum - 1 ? maxX = boundary[0] + range + 1 : maxX = map->ColNum - 1; // max X
	boundary[1] - range - 1 > 0 ? minX = boundary[1] - range - 1 : minX = 0;                             // min X
	boundary[2] + range < map->RowNum - 1 ? maxY = boundary[2] + range : maxY = map->RowNum - 1;     // max Y
	boundary[3] - range > 0 ? minY = boundary[3] - range : minY = 0;                                 // min Y

	// left
	for (int y = minY; y <= maxY; ++y) {
		for (int x = boundary[0]; x <= maxX; ++x) {
			//if (map->map_obstacle(x, y)) break;
			if (map->map_task(x, y)) { // find the task points
				// find which group it belongs to
				int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
				// if not include, include
				vector<int>::iterator iter = find(allstuckGroups.begin(), allstuckGroups.end(), new_groupID);
				if (iter == allstuckGroups.end()) {
					new_neighbors.push_back(new_groupID);
					directions.push_back(left);
					allstuckGroups.push_back(new_groupID);
				}
				//break;
			}
		}
	}
	// right
	for (int y = minY; y <= maxY; ++y) {
		for (int x = boundary[1]; x >= minX; --x) {
			//if (map->map_obstacle(x, y)) break;
			if (map->map_task(x, y)) { // find the task points
				// find which group it belongs to
				int new_groupID = WhichGroup(map->map_task(x, y), taskGroups);
				// if not include, include
				vector<int>::iterator iter = find(allstuckGroups.begin(), allstuckGroups.end(), new_groupID);
				if (iter == allstuckGroups.end()) {
					new_neighbors.push_back(new_groupID);
					directions.push_back(right);
					allstuckGroups.push_back(new_groupID);
				}
				//break;
			}
		}
	}
	cout << "PartNeighbors: \t ";
	for (int k = 0; k < new_neighbors.size(); ++k) {
		cout << new_neighbors[k] << " (" << (*taskGroups)[new_neighbors[k]].leader << "),\t";
	}
	cout << endl;

	vector<vector<int>> neighbors;
	neighbors.push_back(new_neighbors);
	neighbors.push_back(directions);
	return neighbors;
}

// return the task group ID for the input task point
int WaveAlg::WhichGroup(int pointID, vector<TaskSubgroup>* taskGroups) {
	for (int i = 0; i < taskGroups->size(); ++i) {
		for (int j = 0; j < (*taskGroups)[i].ltaskNumber; ++j) {
			if (pointID == (*taskGroups)[i].ltasks[j]->id)
				return i;
		}
		for (int j = 0; j < (*taskGroups)[i].rtaskNumber; ++j) {
			if (pointID == (*taskGroups)[i].rtasks[j]->id)
				return i;
		}
	}
}

// recursion
// input: two groups, one is the stuck group, the other is all stuck group
void WaveAlg::WavePropagation(vector<vector<int>> candidateGroup, vector<int> allstuckGroups, vector<TaskSubgroup>* taskGroups, MatrixMap* map) {
	// 1. find the neighbors
	vector<vector<int>> nextGroups = FindNeighbors(candidateGroup[0], allstuckGroups, taskGroups, map); // neighbors IDs and directions

	cout << "Next stuck groups are (";
	cout << nextGroups[0].size() << "):\t";
	for (int i = 0; i < nextGroups[0].size(); ++i) {
		cout << nextGroups[0][i] << " (" << (*taskGroups)[nextGroups[0][i]].leader << "),\t";
	}
	cout << endl;

	// 2. call self
	if (nextGroups[0].size()) {
		//allstuckGroups.insert(allstuckGroups.end(), nextGroups.begin(), nextGroups.end());
		for (int i = 0; i < nextGroups[0].size(); ++i) {
			allstuckGroups.push_back(nextGroups[0][i]);
		}
		WavePropagation(nextGroups, allstuckGroups, taskGroups, map);
	}

	// 3. move
	for (int i = 0; i < candidateGroup[0].size(); ++i) {
		(*taskGroups)[candidateGroup[0][i]].OverallMove(map, candidateGroup[1][i]); // direction
	}

	return;
}
