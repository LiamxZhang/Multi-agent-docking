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

using namespace std;

class VijayAlg {
public:
	// main function
	void Processing(string data_dir);

	// supporting functions
	void TaskExtension(Task* task, MatrixMap* map);
	void DirectMapping(Task* task);
	void AdaptiveMapping(Task* task, MatrixMap* map);

	// variables
	double taskStep = 0;
	double robotStep = 0;
	double taskStep_mean = 0;
	double robotStep_mean = 0;
	double taskStep_variance = 0;
	double robotStep_variance = 0;
	double taskStep_sys = 0;
	double robotStep_sys = 0;
	bool isComplete = true;
private:
};


void VijayAlg::Processing(string data_dir) {
	// read map, the origin is in the leftmost top,  x means rows, y means columns
	MatrixMap* world = new MatrixMap();
	world->ReadMap(data_dir);
	world->Display("obstacle"); //world.Display();

	// read task, generate assembly tree
	Task* task = new Task();
	task->ReadTask(data_dir);
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
	//AssignTaskToRobot(task, robot);
	HungarianAssign(task, robot, world);

	// show the task extension process
	RecordTaskExtend(task, robot);

	// Robot movement
	robotStep_sys = RobotMove_LocalPlan(task, robot, world);
	if (!robotStep_sys) {
		isComplete = false;
		return;
	}

	//system("pause");
	Recover(task);

	//record the step
	vector<double> steps = RecordStep(task, robot);
	taskStep = steps[0];
	taskStep_mean = steps[1];
	taskStep_variance = steps[2];
	robotStep = steps[3];
	robotStep_mean = steps[4];
	robotStep_variance = steps[5];
}


void VijayAlg::TaskExtension(Task* task, MatrixMap* map) {
	task->PushAll("allExtendedPoints");

	// mapping to the expanded position
	AdaptiveMapping(task, map);
	task->PushAll("allTargets");

	cout << "Start find the targets!" << endl;
	cout << "The center point is: (" << task->centerPoint[0] << ", " << task->centerPoint[1] << ")." << endl << endl;
	// determine all the targets
	int depth = task->AssemblyTree.depth(task->AssemblyTree.root());
	for (int i = depth - 2; i >= 0; i--) {
		// construct the task subgroups
		vector<TaskSubgroup>* taskGroups = new vector<TaskSubgroup>();
		GetTaskSubgroups(taskGroups, task->AssemblyTree.root(), task->SegTree.root(), task, 0, i);

		//cout << endl << endl << "Depth proess:   " << i << endl
		//	<< "How many groups :  " << taskGroups->size() << endl;

		for (int j = 0; j < taskGroups->size(); ++j) {
			//cout << "taskGroup " << j << endl;
			// in the pair, find the closest point to the center
			int closestDist = INT_MAX;
			int closestID;
			vector<int> closestPoint;
			TaskPoint closest;
			// left
			for (int k = 0; k < (*taskGroups)[j].ltaskNumber; ++k) {
				int X = (*taskGroups)[j].ltasks[k]->taskPoint.x;
				int Y = (*taskGroups)[j].ltasks[k]->taskPoint.y;
				int distance = abs(X - task->centerPoint[0]) + abs(Y - task->centerPoint[1]);
				if (distance < closestDist) {
					closestDist = distance;
					closest.id = (*taskGroups)[j].ltasks[k]->id;
					closest.taskPoint.x = X;
					closest.taskPoint.y = Y;
				}
			}

			// right
			for (int k = 0; k < (*taskGroups)[j].rtaskNumber; ++k) {
				int X = (*taskGroups)[j].rtasks[k]->taskPoint.x;
				int Y = (*taskGroups)[j].rtasks[k]->taskPoint.y;
				int distance = abs(X - task->centerPoint[0]) + abs(Y - task->centerPoint[1]);
				if (distance < closestDist) {
					closestDist = distance;
					closest.id = (*taskGroups)[j].rtasks[k]->id;
					closest.taskPoint.x = X;
					closest.taskPoint.y = Y;
				}
			}

			// map to get the current targets
			// delta X = in finalTargets (x - x_closest)
			// mapping X = delta X + in currentTargets x_closest
			// find the closest Index in finalTargets
			int cloest_index;
			for (int k = 0; k < task->taskNum; ++k) {
				if (task->finalTargets[k]->id == closest.id) {
					cloest_index = k;
					break;
				}
			}
			// left
			for (int l = 0; l < (*taskGroups)[j].ltaskNumber; ++l) {
				// find the Indexes of all points in the finalTargets
				int index;
				for (int k = 0; k < task->taskNum; ++k) {
					if (task->finalTargets[k]->id == (*taskGroups)[j].ltasks[l]->id) {
						index = k;
						break;
					}
				}
				// mapping
				int deltaX = task->finalTargets[index]->taskPoint.x - task->finalTargets[cloest_index]->taskPoint.x;
				int deltaY = task->finalTargets[index]->taskPoint.y - task->finalTargets[cloest_index]->taskPoint.y;
				(*taskGroups)[j].ltasks[l]->taskPoint.x = deltaX + closest.taskPoint.x;
				(*taskGroups)[j].ltasks[l]->taskPoint.y = deltaY + closest.taskPoint.y;
			}

			// right
			for (int r = 0; r < (*taskGroups)[j].rtaskNumber; ++r) {
				// find the Indexes of all points in the finalTargets
				int index;
				for (int k = 0; k < task->taskNum; ++k) {
					if (task->finalTargets[k]->id == (*taskGroups)[j].rtasks[r]->id) {
						index = k;
						break;
					}
				}
				// mapping
				int deltaX = task->finalTargets[index]->taskPoint.x - task->finalTargets[cloest_index]->taskPoint.x;
				int deltaY = task->finalTargets[index]->taskPoint.y - task->finalTargets[cloest_index]->taskPoint.y;
				(*taskGroups)[j].rtasks[r]->taskPoint.x = deltaX + closest.taskPoint.x;
				(*taskGroups)[j].rtasks[r]->taskPoint.y = deltaY + closest.taskPoint.y;
			}
		}

		// save
		task->PushAll("allTargets");
		task->PushAll("allExtendedPoints");
	}
	// show the allTargets
	task->Display("all");

	// reajust the allTargets
	cout << endl << "Reajustment!" << endl;
	vector<vector<TaskPoint*>> old_allTargets = task->allTargets;
	vector<vector<TaskPoint*>> new_allTargets;
	for (int i = task->allTargets.size() - 1; i > 0; i--) {
		new_allTargets.push_back(old_allTargets[i]);
	}

	task->allTargets.swap(vector<vector<TaskPoint*>>());
	task->allTargets = new_allTargets;

	// show the allTargets
	task->Display("all");

}


void VijayAlg::DirectMapping(Task* task) {
	int k = 4;
	for (int i = 0; i < task->taskNum; ++i) {
		int centerX = task->centerPoint[0];
		int centerY = task->centerPoint[1];
		int finalX = task->finalTargets[i]->taskPoint.x;
		int finalY = task->finalTargets[i]->taskPoint.y;

		task->currentTargets[i]->taskPoint.x = k * (finalX - centerX) + centerX;
		task->currentTargets[i]->taskPoint.y = k * (finalY - centerY) + centerY;
	}
}

void VijayAlg::AdaptiveMapping(Task* task, MatrixMap* map) {
	DirectMapping(task);

	// find the nearest free space
	for (int i = 0; i < task->taskNum; ++i) {
		int currentX = task->currentTargets[i]->taskPoint.x;
		int currentY = task->currentTargets[i]->taskPoint.y;
		if (map->map_obstacle(currentX, currentY)) { // if obstacle, find other free positions
			int range = 1;
			vector<vector<int>> candidatePoints;

			while (!candidatePoints.size()) {
				int minX, maxX, minY, maxY;
				currentX - range > 0 ? minX = currentX - range : minX = 0;
				currentX + range < map->ColNum - 1 ? maxX = currentX + range : maxX = map->ColNum - 1;
				for (int x = minX; x <= maxX; ++x) {
					int deltaX = x - currentX;
					int deltaY = range - abs(deltaX);
					int y;
					currentY - deltaY > 0 ? y = currentY - deltaY : y = 0;
					if (!map->map_obstacle(x, y) && !map->map_task(x, y)) {
						vector<int> temp;
						temp.push_back(x);
						temp.push_back(y);
						candidatePoints.push_back(temp);
					}

					currentY + deltaY < map->RowNum - 1 ? y = currentY + deltaY : y = map->RowNum - 1;
					if (!map->map_obstacle(x, y) && !map->map_task(x, y)) {
						vector<int> temp;
						temp.push_back(x);
						temp.push_back(y);
						candidatePoints.push_back(temp);
					}
				}
				range += 1;
			}
			// randomly pick one point
			int pick = rand() % candidatePoints.size();
			map->map_task(task->currentTargets[i]->taskPoint.x, task->currentTargets[i]->taskPoint.y) = 0;
			map->map_task(candidatePoints[pick][0], candidatePoints[pick][1]) = task->currentTargets[i]->id;

			task->currentTargets[i]->taskPoint.x = candidatePoints[pick][0];
			task->currentTargets[i]->taskPoint.y = candidatePoints[pick][1];
		}
	}
}
