#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <ctime>
#include <string>
#include <thread>
#include <math.h>
#include <algorithm>
#include <queue>
#include <map>
#include <set>
//#include <mutex>
#include <windows.h>

#include "Point.h"
#include "Task.h"
#include "Map.h"
#include "Robot.h"
using namespace std;

//void ExtendTask(BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, MatrixMap* map);
void ExtendTask(BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, MatrixMap* map, int depth, int obj);
void ExtendAction(BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, MatrixMap* map);
bool RecordTaskPosition(vector<TaskPoint*> allTargets);

int main() {
	// read map, the origin is in the leftmost top,  x means rows, y means columns
	MatrixMap* world = new MatrixMap();
	world->ReadMap();
	//world.Display();

	// read task, generate assembly tree
	Task* task = new Task();
	task->ReadTask();
	task->GenerateTree();
	//cout << endl << "Depth: " << task->AssemblyTree.depth(task->AssemblyTree.root()) << endl;
	//cout << endl << world->TaskCheck(1, task->AssemblyTree.leaves()[0]->data, 2) << endl; // 检查地图有效

	// move the task components, according to the assembly tree
	//ExtendTask(task->AssemblyTree.root(),task->SegTree.root(),task,world);
	int depth = task->AssemblyTree.depth(task->AssemblyTree.root());
	for (int i = 0; i < depth-1; i++) { // assembly tree 的最后一层只有一个机器人不必进行
		//
		ExtendTask(task->AssemblyTree.root(), task->SegTree.root(), task, world, 0, i);
		// 
		vector<TaskPoint*> tempTargets;
		for (int i = 0; i < task->currentTargets.size(); i++) {
			TaskPoint* temp = new TaskPoint();
			temp->id = task->currentTargets[i]->id;
			temp->taskPoint.x = task->currentTargets[i]->taskPoint.x;
			temp->taskPoint.y = task->currentTargets[i]->taskPoint.y;
			tempTargets.push_back(temp);
		}
		task->allTargets.push_back(tempTargets);
		// display
		cout << endl;
		world->Display();
		cout << endl;
	}
	
	// display task positions of steps
	for (int i = 0; i < task->allTargets.size(); i++) {
		cout << "target step " << i << ": ";  // i denotes the ith step, j denotes the jth robot
		for (int j = 0; j < task->allTargets[0].size(); j++)
			cout << "("<< task->allTargets[i][j]->id << ",  " << task->allTargets[i][j]->taskPoint.x
			<< ", " << task->allTargets[i][j]->taskPoint.y << "), ";
		cout << endl;
	}

	/*
	// write into file
	vector<TaskPoint*> allTargets;
	for (int i = 0; i < task->allTargets.size(); i++) {
		TaskPoint* temp = new TaskPoint();
		temp = task->allTargets.back()[i];
		//task->allTargets[i].pop_back();
		allTargets.push_back(temp);
		cout << temp->id << " " << temp->taskPoint.x << " "
			<< temp->taskPoint.y << endl;
	}
	RecordTaskPosition(allTargets);
	*/
	// create the robots
	vector<Robot*> robot;
	for (int i = 0; i < task->robotNum; i++) {
		Robot* temp = new Robot(task->robotNum,task->startPoints[i]->id, task->startPoints[i]->taskPoint);
		temp->ReadMap();
		robot.push_back(temp);
	}
	// assign the task to the closest robots using optimization (or bid)
	// from task->allTargets[j][i]->taskpoint.x(y)
	// to robot[i]->initPosition.x(y)
	float closestDistance;
	float distance;
	bool assigned;
	vector<int> assignedTaskID;
	for (int i = 0; i < task->robotNum; i++) {
		closestDistance = abs(world->RowNum) + abs(world->ColNum) + 10;   // max is the whole map size
		int roboX = robot[i]->initPosition.x;
		int roboY = robot[i]->initPosition.y;
		for (int j = 0; j < task->taskNum; j++) {
			// if task has been assigned, break
			assigned = false;
			for (int k = 0; k < assignedTaskID.size(); k++)
				if (assignedTaskID[k] == task->allTargets.back()[j]->id) {
					assigned = true;
					break;
				}
			if (assigned) continue;
			// calculate the distance
			int taskX = task->allTargets.back()[j]->taskPoint.x;
			int taskY = task->allTargets.back()[j]->taskPoint.y;
			distance = abs(roboX - taskX) + abs(roboY - taskY); // Manhattan distance
			//
			//cout << endl << "X: " << roboX << "    Y: " << roboY << "   distance: " << distance;
			if (distance < closestDistance) {
				closestDistance = distance;
				robot[i]->taskID = task->allTargets.back()[j]->id;
			}
		}
		assignedTaskID.push_back(robot[i]->taskID);
	}
	cout << endl;
	for (int i = 0; i < task->robotNum; i++) {
		cout << "Assign robot " << robot[i]->id << ": task ID " << robot[i]->taskID << endl;
	}
	cout << endl;
	// path planning based on the rule->ShortestPath
	// collision avoidance
	// move the robot
	// component移动时保证在一起

	int stepNum = task->allTargets.size(); // 应等于tree的深度
	int roboNum = task->allTargets[0].size();  // task number = robot number
	// task ID -> robot ID -> robot index
	vector<int> tID2index;
	vector<int> rID2index;
	for (int i = 0; i <= roboNum; i++) { // task ID : 1~robot number
		int taskID = i;
		int index = 0;
		for (int j = 0; j < roboNum; j++)
			if (robot[j]->taskID == taskID) {  index = j;  break; }
		tID2index.push_back(index);
		cout << "task ID: " << taskID << "   index: " << index << endl;
		int robotID = i;
		int ind = 0;
		for (int j = 0; j < roboNum; j++)
			if (robot[j]->id == robotID) { ind = j;  break; }
		rID2index.push_back(ind);
		cout << "robot ID: " << robotID << "   index: " << ind << endl;
	}

	for (int i = 0; i < stepNum; i++) { // for each one layer
		vector<TaskPoint*> allTargets = task->allTargets[stepNum-i-1]; // get task points of all robots at the last time
		cout << "allTargets:  ";
		for (int j = 0; j < roboNum; j++) {
			cout << "("<< allTargets[j]->id << ", " << allTargets[j]->taskPoint.x << ", "
				<< allTargets[j]->taskPoint.y << "),";
		}
		cout << endl;
		//RecordTaskPosition(allTargets);
		
		// assign leader, task->AssemblyTree
		vector<int> leaders; // all the leaders' robot IDs
		vector<BinNode<vector<int>>*> nodeVec; // all nodes of task tree
		nodeVec = task->AssemblyTree.getLayerNode(task->AssemblyTree.root(), 0, stepNum - i - 1, nodeVec);
		// see the nodes
		cout << "all nodes: ";
		for (int j = 0; j < nodeVec.size(); j++) {
			cout << "(";
			for (int k = 0; k < nodeVec[j]->data.size(); k++) {
				cout << nodeVec[j]->data[k] << ",";
			}
			cout << "),";
		}
		cout << endl;
		for (int j = 0; j < nodeVec.size(); j++) { // for each one node 
			// find the minimum robot ID value as the leader
			int min = 10000; // robot leader ID
			for (int k = 0; k < nodeVec[j]->data.size(); k++) {
				int taskID = nodeVec[j]->data[k];
				if (robot[tID2index[taskID]]->id < min)
					min = robot[tID2index[taskID]]->id;
				cout << "ID: " << robot[tID2index[taskID]]->id << endl;
			}
			cout << "Leader: " << min << endl;
			leaders.push_back(min);
			
			for (int k = 0; k < nodeVec[j]->data.size(); k++) {
				// assign the leader value (robot ID)
				int taskID = nodeVec[j]->data[k];
				robot[tID2index[taskID]]->leader = min;
				// build the mapping relation inside one component between the leader and the follower // leader + offset = robot position
				robot[tID2index[taskID]]->offset.swap(vector<int>());
				robot[tID2index[taskID]]->offset.push_back(robot[tID2index[taskID]]->currentPosition.x - robot[rID2index[min]]->currentPosition.x);
				robot[tID2index[taskID]]->offset.push_back(robot[tID2index[taskID]]->currentPosition.y - robot[rID2index[min]]->currentPosition.y);
			}

			// path planning for the leaders // robot: update workmap, weightMap; 

			// update robot targetPosition
			leaders is min;
			vector<Point> planPath;
			planPath = robot[0]->ShortestPath(robot[0]->initPosition, robot[5]->initPosition);

			cout << endl << "Path: ";
			for (int i = 0; i < planPath.size(); i++) {
				cout << planPath[i].x << " and " << planPath[i].y << endl;
			}
		
		}
		for (int i = 0; i < roboNum; i++) 
			cout << "robot " << robot[i]->id << " task: " << robot[i]->taskID << " leader: " << robot[i]->leader << "  offset: " << robot[i]->offset[0] << ", " << robot[i]->offset[1] << endl;

		

		// map the followers

		// move one step, check collision

		// if collision, component with higher priority continues to move
		// the other one add the new obstacle and replanning

		// if free, continue to move

		system("pause");
	}

	/*
	// test path planning
	vector<Point> planPath;
	planPath = robot[0]->ShortestPath(robot[0]->initPosition, robot[5]->initPosition);
	cout << endl << "Path: ";
	for (int i = 0; i < planPath.size(); i++) {
		cout << planPath[i].x << " and " << planPath[i].y << endl;
	}
	// test get layer number
	vector<int> NumPerLayer = task->AssemblyTree.countNumPerLayer();
	cout << "number per layer: ";
	for (int i = 0; i < NumPerLayer.size(); i++) {
		cout << NumPerLayer[i] << ",";
	}
	cout << endl;
	// test get layer nodes
	vector<BinNode<vector<int>>*> nodeVec;
	nodeVec = task->AssemblyTree.getLayerNode(task->AssemblyTree.root(), 0, 2, nodeVec);
	cout << "Layer nodes are: ";
	for (int i = 0; i < nodeVec.size(); i++) {
		cout << "(";
		for (int j = 0; j < nodeVec[i]->data.size(); j++) {
			cout << nodeVec[i]->data[j] << ", ";
		}
		cout << "), ";
	}
	cout << endl;
	*/

	system("pause");
	return 0;
}


// functions
// input: task, map, rule
// task: assembly tree, segmentation tree
// map: check collision
/*
void ExtendTask(BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, MatrixMap* map) {
	// end condition
	if (!assNode) return;   // tree node empty
	if (assNode->data.size() <= 1) return;  // cannot be extended anymore

	// find the position of target id in currentTargets
	vector<int> lcomponents = assNode->lChild->data; 
	vector<int> left_ids;
	for (int i = 0; i < lcomponents.size(); i++)
		for (int j = 0; j < task->currentTargets.size(); j++)
			if (task->currentTargets[j]->id == lcomponents[i])    // ids[i] <-> components[i]
				left_ids.push_back(j);
	//
	vector<int> rcomponents = assNode->rChild->data;
	vector<int> right_ids;
	for (int i = 0; i < rcomponents.size(); i++)
		for (int j = 0; j < task->currentTargets.size(); j++)
			if (task->currentTargets[j]->id == rcomponents[i])    // ids[i] <-> components[i]
				right_ids.push_back(j);

	// 移动一步，记录一次
	int step = 2; // 步长
	bool done = false;   // flag indicates the movement finish
	int total_move = 0;
	
	while (!done) {
		if (segNode->data == 'x') {   // 沿x方向分离  // lcomponent的targets点都x+1, rcomponent都x-1
			{// left
				done = true;  // 是否 没有障碍
				vector<TaskPoint*> tempTargets(task->currentTargets); // 佯装移动
				for (int i = 0; i < lcomponents.size(); i++) {
					cout << " xl " << i << endl;
					int temp_pos = tempTargets[left_ids[i]]->taskPoint.x + step;
					// collision, check map. 遇到任务点与遇到障碍物的反应应当不同
					if (map->TaskCheck(temp_pos, tempTargets[left_ids[i]]->taskPoint.y, lcomponents, step))  // 如果没有障碍
						tempTargets[left_ids[i]]->taskPoint.x = temp_pos;
					else   // 如果附近有障碍
						done = false;
				}
				
				if (done) {  // 如果整个component都没有遇到障碍
					task->currentTargets.assign(tempTargets.begin(), tempTargets.end());
					total_move += step;
					// update map
					map->UpdateTaskmap(tempTargets);
				}
			}
			{// right
				done = true;  // 是否 没有障碍
				vector<TaskPoint*> tempTargets(task->currentTargets);
				for (int i = 0; i < rcomponents.size(); i++) {
					cout << " rl " << i << endl;
					int temp_pos = tempTargets[right_ids[i]]->taskPoint.x - step;
					//
					if (map->TaskCheck(temp_pos, tempTargets[left_ids[i]]->taskPoint.y, rcomponents, step))   // 如果没有障碍
						tempTargets[right_ids[i]]->taskPoint.x = temp_pos;
					else   // 如果有障碍
						done = false;
				}
				if (done) {  // 如果整个component都没有遇到障碍
					task->currentTargets.assign(tempTargets.begin(), tempTargets.end());
					total_move += step;
					// update map
					map->UpdateTaskmap(tempTargets);
				}
			}
		}
		else if (segNode->data == 'y') {  // 沿y方向分离
			{// left
				done = true;  // 是否 没有障碍
				vector<TaskPoint*> tempTargets(task->currentTargets); // 佯装移动
				for (int i = 0; i < lcomponents.size(); i++) {
					cout << " yl " << i << endl;
					int temp_pos = tempTargets[left_ids[i]]->taskPoint.y + step;
					// collision, check map. 遇到任务点与遇到障碍物的反应应当不同
					if (map->TaskCheck(tempTargets[left_ids[i]]->taskPoint.x, temp_pos, lcomponents, step))   // 如果没有障碍
						tempTargets[left_ids[i]]->taskPoint.y = temp_pos;
					else   // 如果有障碍
						done = false;
				}
				if (done) {  // 如果整个component都没有遇到障碍
					task->currentTargets.assign(tempTargets.begin(), tempTargets.end());
					total_move += step;
					// update map
					map->UpdateTaskmap(tempTargets);
				}
			}
			{// right
				done = true;  // 是否 没有障碍
				vector<TaskPoint*> tempTargets(task->currentTargets);
				for (int i = 0; i < rcomponents.size(); i++) {
					cout << " yr " << i << endl;
					int temp_pos = tempTargets[right_ids[i]]->taskPoint.y - step;
					//
					if (map->TaskCheck(tempTargets[left_ids[i]]->taskPoint.x, temp_pos, rcomponents, step))   // 如果没有障碍
						tempTargets[right_ids[i]]->taskPoint.y = temp_pos;
					else   // 如果有障碍
						done = false;
				}
				if (done) {  // 如果整个component都没有遇到障碍
					task->currentTargets.assign(tempTargets.begin(), tempTargets.end());
					total_move += step;
					// update map
					map->UpdateTaskmap(tempTargets);
				}
			}
		}
		// else segNode containes nonsenses
		// end condition: if displacement >= Num, done = true
		if (total_move >= step * 2) break;
		else done = false;
	}
	if (done) {// displacement 足够远,更新alltargets
		vector<TaskPoint*> tempTargets;
		for (int i = 0; i < task->currentTargets.size(); i++) {
			TaskPoint* temp = new TaskPoint();
			temp->id = task->currentTargets[i]->id;
			temp->taskPoint.x = task->currentTargets[i]->taskPoint.x;
			temp->taskPoint.y = task->currentTargets[i]->taskPoint.y;
			tempTargets.push_back(temp);
		}
		task->allTargets.push_back(tempTargets);
	}
	// display
	cout << endl; // monitor allTargets
	for (int i = 0; i < task->allTargets.size(); i++) {
		cout << "Current step " << i << ": ";  // i denotes the ith step, j denotes the jth robot
		for (int j = 0; j < task->allTargets[0].size(); j++)
			cout << "(" << task->allTargets[i][j]->id << ",  " << task->allTargets[i][j]->taskPoint.x
			<< ", " << task->allTargets[i][j]->taskPoint.y << "), ";
		cout << endl;
	}
	cout << endl;
	map->Display();
	cout << endl;
	// recursion resumes
	ExtendTask(assNode->lChild, segNode->lChild, task, map);
	ExtendTask(assNode->rChild, segNode->rChild, task, map);
}
*/

void ExtendTask(BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, MatrixMap* map, int depth, int obj) {
	// end condition
	if (!assNode) return;   // tree node empty
	if (assNode->data.size() <= 1) return;  // cannot be extended anymore

	if (depth == obj) {
		ExtendAction(assNode, segNode, task, map);
	}
	ExtendTask(assNode->lChild, segNode->lChild, task, map, depth + 1, obj);
	ExtendTask(assNode->rChild, segNode->rChild, task, map, depth + 1, obj);
}


void ExtendAction(BinNode<vector<int>>* assNode, BinNode<char>* segNode, Task* task, MatrixMap* map) {
	// find the position of target id in currentTargets
	vector<int> lcomponents = assNode->lChild->data;
	vector<int> left_ids;
	for (int i = 0; i < lcomponents.size(); i++)
		for (int j = 0; j < task->currentTargets.size(); j++)
			if (task->currentTargets[j]->id == lcomponents[i])    // ids[i] <-> components[i]
				left_ids.push_back(j);
	//
	vector<int> rcomponents = assNode->rChild->data;
	vector<int> right_ids;
	for (int i = 0; i < rcomponents.size(); i++)
		for (int j = 0; j < task->currentTargets.size(); j++)
			if (task->currentTargets[j]->id == rcomponents[i])    // ids[i] <-> components[i]
				right_ids.push_back(j);

	// 移动一步，记录一次, 探测距离=截止距离
	int step = 1; // 步长
	int detect_dist = 2;  // 探测距离
	bool done = false;   // flag indicates the movement finish
	int total_move = 0;  // 单次总移动距离

	while (!done) {
		if (segNode->data == 'x') {   // 沿x方向分离  // lcomponent的targets点都x+1, rcomponent都x-1
			{// left
				done = true;  // 是否 没有障碍
				vector<TaskPoint*> tempTargets(task->currentTargets); // 佯装移动
				for (int i = 0; i < lcomponents.size(); i++) {   // component移动
					cout << " xl " << i << endl;
					int temp_pos = tempTargets[left_ids[i]]->taskPoint.x + step;
					// collision, check map. 遇到任务点与遇到障碍物的反应应当不同
					if (map->TaskCheck(temp_pos, tempTargets[left_ids[i]]->taskPoint.y, assNode->data, detect_dist))  // 如果没有障碍
						tempTargets[left_ids[i]]->taskPoint.x = temp_pos;
					else   // 如果附近有障碍
						done = false;
				}

				if (done) {  // 如果整个component都没有遇到障碍
					task->currentTargets.assign(tempTargets.begin(), tempTargets.end());
					total_move += step;
					// update map
					map->UpdateTaskmap(tempTargets);
				}
			}
			{// right
				done = true;  // 是否 没有障碍
				vector<TaskPoint*> tempTargets(task->currentTargets);
				for (int i = 0; i < rcomponents.size(); i++) {
					cout << " rl " << i << endl;
					int temp_pos = tempTargets[right_ids[i]]->taskPoint.x - step;
					//
					if (map->TaskCheck(temp_pos, tempTargets[right_ids[i]]->taskPoint.y, assNode->data, detect_dist))   // 如果没有障碍
						tempTargets[right_ids[i]]->taskPoint.x = temp_pos;
					else   // 如果有障碍
						done = false;
				}
				if (done) {  // 如果整个component都没有遇到障碍
					task->currentTargets.assign(tempTargets.begin(), tempTargets.end());
					total_move += step;
					// update map
					map->UpdateTaskmap(tempTargets);
				}
			}
		}
		else if (segNode->data == 'y') {  // 沿y方向分离
			{// left
				done = true;  // 是否 没有障碍
				vector<TaskPoint*> tempTargets(task->currentTargets); // 佯装移动
				for (int i = 0; i < lcomponents.size(); i++) {
					cout << " yl " << i << endl;
					int temp_pos = tempTargets[left_ids[i]]->taskPoint.y + step;
					// collision, check map. 遇到任务点与遇到障碍物的反应应当不同
					if (map->TaskCheck(tempTargets[left_ids[i]]->taskPoint.x, temp_pos, assNode->data, detect_dist))   // 如果没有障碍
						tempTargets[left_ids[i]]->taskPoint.y = temp_pos;
					else   // 如果有障碍
						done = false;
				}
				if (done) {  // 如果整个component都没有遇到障碍
					task->currentTargets.assign(tempTargets.begin(), tempTargets.end());
					total_move += step;
					// update map
					map->UpdateTaskmap(tempTargets);
				}
			}
			{// right
				done = true;  // 是否 没有障碍
				vector<TaskPoint*> tempTargets(task->currentTargets);
				for (int i = 0; i < rcomponents.size(); i++) {
					cout << " yr " << i << endl;
					int temp_pos = tempTargets[right_ids[i]]->taskPoint.y - step;
					//
					if (map->TaskCheck(tempTargets[right_ids[i]]->taskPoint.x, temp_pos, assNode->data, detect_dist))   // 如果没有障碍
						tempTargets[right_ids[i]]->taskPoint.y = temp_pos;
					else   // 如果有障碍
						done = false;
				}
				if (done) {  // 如果整个component都没有遇到障碍
					task->currentTargets.assign(tempTargets.begin(), tempTargets.end());
					total_move += step;
					// update map
					map->UpdateTaskmap(tempTargets);
				}
			}
		}
		// else segNode containes nonsenses
		// end condition: if displacement >= Num, done = true
		if (total_move >= step * 2) break;
		else done = false;
	}
	/*
	if (done) {// assembly tree的一层完成,更新alltargets
		vector<TaskPoint*> tempTargets;
		for (int i = 0; i < task->currentTargets.size(); i++) {
			TaskPoint* temp = new TaskPoint();
			temp->id = task->currentTargets[i]->id;
			temp->taskPoint.x = task->currentTargets[i]->taskPoint.x;
			temp->taskPoint.y = task->currentTargets[i]->taskPoint.y;
			tempTargets.push_back(temp);
		}
		task->allTargets.push_back(tempTargets);
	}
	// display
	cout << endl;
	map->Display();
	cout << endl;
	*/
}

// write the task point into txt file
bool RecordTaskPosition(vector<TaskPoint*> allTargets) {
	// allTargets is the vector of the current task points
	ofstream f;
	f.open("../TestRobot/Task.txt", ofstream::out);
	if (f) {
		f << allTargets.size() << endl;
		for (int i = 0; i < allTargets.size(); i++) {
			//
			f << allTargets[i]->id << " " << allTargets[i]->taskPoint.x << " "
				<< allTargets[i]->taskPoint.y << endl;
			// Log
		}
		return true;
	}
	else return false;
}

/*
// record the current position of robots
bool RecordCurrentAndTargetPosition(int size) {
	ofstream f;
	f.open("../TestRobot/Robot_Current_Position.txt", ofstream::out);
	if (f) {
		f << robotCurrentPosition.size() << endl;
		RecordLog("RecordCurrentAndTargetPosition:");
		RecordLog("RobotId   CurrentPosition   TargetPosition");
		for (int i = 1; i <= robotCurrentPosition.size(); ++i) {
			f << i << "," << robotCurrentPosition[i - 1].x + 1 << "," << robotCurrentPosition[i - 1].y + 1 << ","
				<< robotTargetPosition[i - 1].x + 1 << "," << robotTargetPosition[i - 1].y + 1 << endl;
			RecordLog("   " + to_string(i) + "          [" + to_string(robotCurrentPosition[i - 1].x + 1) + ","
				+ to_string(robotCurrentPosition[i - 1].y + 1) + "]              [" + to_string(robotTargetPosition[i - 1].x + 1) +
				"," + to_string(robotTargetPosition[i - 1].y + 1) + "]");
		}
	}
	else
		return false;
	f.close();
	return true;
}

*/