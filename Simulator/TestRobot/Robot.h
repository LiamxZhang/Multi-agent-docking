#pragma once
// 定义单个机器人的数据格式
// 存储机器人在地图中的所有信息
// weightPoint 定义机器人四邻域的状态
//
#include <iostream>
#include <vector>

#include "Map.h"
#include "Point.h"
#include "Log.h"

using namespace std;

//权重点类的定义
class WeightPoint {           // 每个机器人四邻域的值，1是能走，-1是不能走
private:
	int up;
	int down;
	int left;
	int right;
public:
	WeightPoint() :up(0), down(0), left(0), right(0) {}
	bool operator == (const WeightPoint& r) const {
		return (up == r.up) && (down == r.down) && (left == r.left) && (right == r.right);
	}
	friend class Rule;
	friend class Robot;
	friend int main();
};

class Robot {
public:
	int robotNumber;           //robot number
	int id;                    // robot Id , start from zero
	int taskID;
	int leader;                // leader ID

	// robot position
	Point initPosition;       // initPosition of robot, used for assigning task
	Point currentPosition;    // currentPosition of robot
	Point tendPosition;       // Position of next one step
	Point targetPosition;     // targetPosition of robot move

	// robot move path
	vector<Point> planPath;   // the planned path of robot, current walking on
	vector<Point> robotPosition;   //机器人走过的一系列位置
	vector<int> offset;      // position offset to the leader (delta x, delta y)

	// robot map
	int mapRowNumber;         // the row number of map
	int mapColumnNumber;      // the column number of map
	vector<vector<int>> workmap;  // the map robot walks
	vector<vector<WeightPoint>> weightMap;    // the map robot think the map

	Robot() :id(0), mapRowNumber(0), mapColumnNumber(0), initPosition(), currentPosition(), targetPosition() {
		++robotNumber;
		cout << "Construct Robot Number " << robotNumber << endl;
		//RecordLog("Construct Robot Number " + to_string(robotNumber));
	}
	Robot(int robotnum, int ID, Point position) : robotNumber(robotnum), id(ID), 
		mapRowNumber(0), mapColumnNumber(0), currentPosition(), targetPosition() {
		initPosition.x = position.x;
		initPosition.y = position.y;
		currentPosition.x = position.x;
		currentPosition.y = position.y;
		//cout << "Construct Robot Number " << robotNumber << endl;
		//RecordLog("Construct Robot Number " + to_string(robotNumber));
	}
	// robot get the total robot number, its id and initial position
	bool ReadMap();
	vector<Point> AStarPath();   // plan the route of robot, from currentPosition to targetPosition
	vector<Point> MappingPath(vector<Point>);  // mapping the path from leader's path
	void UpdateMap(MatrixMap*, vector<int>);    // update workmap and weightMap
	bool TrialStep();
	void OneStep();         // robot move

	friend int main();
	friend class Rule;
};

bool
Robot::ReadMap() {    // Read the map from given file and form the weight map
	ifstream f;
	f.open("../TestRobot/InitMap.txt", ifstream::in);
	if (f) {
		f >> mapRowNumber;
		char c;
		f >> c;
		f >> mapColumnNumber;
		for (int r = 0; r < mapRowNumber; ++r) {
			vector<int> oneRow;
			for (int c = 0; c < mapColumnNumber; ++c) {
				int onepoint;
				char t;
				f >> onepoint;
				oneRow.push_back(onepoint);
				if (c != (mapColumnNumber - 1))
					f >> t;//t may be the next-line signal
			}
			//map.push_back(oneRow);
			workmap.push_back(oneRow);
		}
	}
	else {
		cout << "Read Robot: Failed to open the InitMap.txt! " << endl;
		RecordLog("Read Robot: Failed to open the InitMap.txt!");
		return false;
	}
	f.close();
	//
	for (int i = 0; i < mapRowNumber; ++i) {
		vector<WeightPoint> oneRow;
		for (int j = 0; j < mapColumnNumber; ++j) {
			WeightPoint t;
			if (workmap[i][j] == 1) {
				t.up = -1;
				t.down = -1;
				t.left = -1;
				t.right = -1;
			}
			else {
				if ((i - 1) >= 0 && (i - 1) < mapRowNumber) {
					if (workmap[i - 1][j] == 0)
						t.up = 1;
					else
						t.up = -1;
				}
				else
					t.up = -1;
				if ((i + 1) >= 0 && (i + 1) < mapRowNumber) {
					if (workmap[i + 1][j] == 0)
						t.down = 1;
					else
						t.down = -1;
				}
				else
					t.down = -1;
				if ((j - 1) >= 0 && (j - 1) < mapColumnNumber) {
					if (workmap[i][j - 1] == 0)
						t.left = 1;
					else
						t.left = -1;
				}
				else
					t.left = -1;
				if ((j + 1) >= 0 && (j + 1) < mapColumnNumber) {
					if (workmap[i][j + 1] == 0)
						t.right = 1;
					else
						t.right = -1;
				}
				else
					t.right = -1;
			}
			oneRow.push_back(t);
		}
		weightMap.push_back(oneRow);
	}
	//cout << "Success to read the map from file InitMap.txt" << endl;
	//RecordLog("Success to read the map from file InitMap.txt");
	return true;
}

vector<Point>
Robot::AStarPath() {   // find a route from currentPosition to targetPosition
	Point& source = currentPosition;
	Point& target = targetPosition;
	vector<Point> path;

	if (workmap[source.x][source.y] == 1 || workmap[target.x][target.y] == 1) {
		cout << "起点或终点有障碍物" << endl;
	}

	if (source == target) {
		path.push_back(source);
		return path;
	}
	// A* algorithm

	struct APoint {
		Point p;
		double g;
		double f;
		bool operator < (const APoint& a) const {
			return f < a.f;  // 最小值优先
		}
	};

	vector<Point> closeList;
	vector<APoint> openList;
	map<Point, Point> relation;
	int choice = 0;
	APoint ap;
	ap.p = source;
	ap.g = 0;
	ap.f = 0;
	openList.push_back(ap);
	double g = 0;
	double h = 0;
	double f = 0;
	double t = 0;
	bool flag = true;
	while (openList.empty() == false && flag == true) {
		ap = openList[0];
		openList.erase(openList.begin());
		Point cur = ap.p;
		//cur = Avoid(cur);
		closeList.push_back(cur);
		Point next;
		double gn = 0;


		for (int direct = 1; direct <= 4; ++direct) {
			if (direct == 1) {
				next = cur.up();
				gn = weightMap[ap.p.x][ap.p.y].up;
			}
			else if (direct == 2) {
				next = cur.right();
				gn = weightMap[ap.p.x][ap.p.y].right;
			}
			else if (direct == 3) {
				next = cur.down();
				gn = weightMap[ap.p.x][ap.p.y].down;
			}
			else if (direct == 4) {
				next = cur.left();
				gn = weightMap[ap.p.x][ap.p.y].left;
			}
			if (next.x >= 0 && next.x < mapRowNumber && next.y >= 0 &&
				next.y < mapColumnNumber && workmap[next.x][next.y] == 0) {
				//计算新的F(N)

				int ci = -1;
				for (int i = 0; i < closeList.size(); ++i) {
					if (closeList[i] == next) {
						ci = i;
						break;
					}
				}

				if (ci == -1) {
					int oi = -1;
					for (int i = 0; i < openList.size(); ++i) {
						if (openList[i].p == next) {
							oi = i;
							break;
						}
					}
					if (oi == -1) {    //openList中不存在遍历节点
						g = ap.g + 1 + gn;
						h = abs(target.x - next.x) + abs(target.y - next.y);
						if (choice == 0) {
							t = 1;
						}
						else if (choice == 1) {
							t = 1.0 + 1.0 / (sqrt(1.0 * (target.x - next.x) * (target.x - next.x) +
								(target.y - next.y) * (target.y - next.y)));
						}
						f = g + h * t;
						APoint ap1;
						ap1.p = next;
						ap1.g = g;
						ap1.f = f;
						openList.push_back(ap1);
						relation[next] = cur;
					}
					else {           //openList中存在遍历节点
						g = ap.g + 1 + gn;
						if (g < openList[oi].g) {
							openList[oi].g = g;
							h = abs(target.x - next.x) + abs(target.y - next.y);
							if (choice == 0) {
								t = 1;
							}
							else if (choice == 1) {
								t = 1.0 + 1.0 / (sqrt(1.0 * (target.x - next.x) * (target.x - next.x) +
									(target.y - next.y) * (target.y - next.y)));
							}
							f = openList[oi].g + h * t;
							openList[oi].f = f;
							relation[next] = cur;
						}
					}
				}
			}
		}

		if (cur == target) {
			flag = false;
			break;
		}
		sort(openList.begin(), openList.end());
	}

	//vector<Point> result;
	planPath.swap(vector<Point>());
	planPath.push_back(target);
	Point cur = target;
	Point next = relation[cur];
	while (!(relation[cur] == source)) {
		planPath.push_back(relation[cur]);
		cur = relation[cur];
	}

	planPath.push_back(source);
	int low = 0, high = planPath.size() - 1;
	while (low < high) {
		Point tmp = planPath[low];
		planPath[low] = planPath[high];
		planPath[high] = tmp;
		++low;
		--high;
	}
	return planPath;
}

vector<Point>
Robot::MappingPath(vector<Point> leaderPath) {   // prerequest offset and leader's path
	planPath.swap(vector<Point>());
	for (int i = 0; i < leaderPath.size(); ++i) {
		Point temp;
		temp.x = leaderPath[i].x + offset[0];
		temp.y = leaderPath[i].y + offset[1];
		planPath.push_back(temp);
	}
	return planPath;
}

void 
Robot::UpdateMap(MatrixMap* world, vector<int> peer) {
	// clear the maps
	workmap.swap(vector<vector<int>>());
	weightMap.swap(vector<vector<WeightPoint>>());
	// update workmap
	for (int r = 0; r < mapRowNumber; ++r) {
		vector<int> oneRow;
		for (int c = 0; c < mapColumnNumber; ++c) {
			if (world->map_obstacle(r, c))  // if obstacle, 1
				oneRow.push_back(1);
			else if (world->map_robot(r, c) != 0 && world->map_robot(r, c) != id) {  // if other robots, 2
				bool isPeer = false;
				for (int i = 0; i < peer.size(); ++i) if (peer[i] == world->map_robot(r, c)) isPeer = true;
				(isPeer)? oneRow.push_back(0): oneRow.push_back(2);
			}
			else
				oneRow.push_back(0);
		}
		workmap.push_back(oneRow);
	}
	// update current weightMap
	for (int i = 0; i < mapRowNumber; ++i) {
		vector<WeightPoint> oneRow;
		for (int j = 0; j < mapColumnNumber; ++j) {
			WeightPoint t;
			if (workmap[i][j] != 0) {
				t.up = -1;
				t.down = -1;
				t.left = -1;
				t.right = -1;
			}
			else {
				((i - 1) >= 0 && (i - 1) < mapRowNumber)? t.up = 1 - workmap[i - 1][j]: t.up = -1;
				((i + 1) >= 0 && (i + 1) < mapRowNumber)? t.down = 1 - workmap[i + 1][j]: t.down = -1;
				((j - 1) >= 0 && (j - 1) < mapColumnNumber)? t.left = 1 - workmap[i][j - 1]: t.left = -1;
				((j + 1) >= 0 && (j + 1) < mapColumnNumber)? t.left = 1 - workmap[i][j + 1]: t.right = -1;
			}
			oneRow.push_back(t);
		}
		weightMap.push_back(oneRow);
	}
}

bool 
Robot::TrialStep() {   // change tendPosition
	if (planPath.size() > 1) {
		tendPosition.x = planPath[1].x;
		tendPosition.y = planPath[1].y;
		//cout << endl << "Robot " << id << " trial step: (" << planPath[0].x << ", " << planPath[0].y << ");     " << endl;
		return true;
	}
	else {  // planPath.size() == 0
		tendPosition.x = currentPosition.x;
		tendPosition.y = currentPosition.y;
		return false;
	}
}

void
Robot::OneStep() {
	currentPosition.x = tendPosition.x;
	currentPosition.y = tendPosition.y;
}

/////////////////////////////////////////////////////////
class RobotGroup {
public:
	int robotNumber;   // robot number in this group
	int leader;        // leader robot ID
	vector<Robot*> robot; // group members

	RobotGroup(vector<Robot*> r) : robotNumber(r.size()), robot(r){}
	bool AssignLeaders();
	void Display() { for (int k = 0; k < robot.size(); k++)   cout << robot[k]->id << ", "; }
	void PathPlanning(MatrixMap*, vector<TaskPoint*>);
	void TrialMove();
	void Move(MatrixMap*);
	vector<Point> GetRobotPos();
	vector<int> GetRobotIds();
};

// assign the leader value (robot ID)
// build the mapping relation inside one component between the leader and the follower 
// leader + offset = robot position
bool 
RobotGroup::AssignLeaders() {
	if (!robot.size()) return false;
	// find the leader robot ID value
	int leaderID = robot[0]->id;

	// assign
	for (int k = 0; k < robot.size(); k++) {
		// assign the leader value (robot ID)
		robot[k]->leader = leaderID;
		// build the mapping relation inside one component between the leader and the follower // leader + offset = robot position
		robot[k]->offset.swap(vector<int>());
		robot[k]->offset.push_back(robot[k]->currentPosition.x - robot[0]->currentPosition.x);
		robot[k]->offset.push_back(robot[k]->currentPosition.y - robot[0]->currentPosition.y);
	}
	// see the assignments
	/*
	for (int i = 0; i < robotNumber; i++)
		cout << "robot " << robot[i]->id << " task: " << robot[i]->taskID << " leader: " 
		<< robot[i]->leader << "  offset: " << robot[i]->offset[0] << ", " << robot[i]->offset[1] << endl;
	*/
	return true;
}

// update the robot workmap, weightMap and targetPosition
void 
RobotGroup::PathPlanning(MatrixMap* world, vector<TaskPoint*> allTargets) {
	// leader update robot workmap, weightMap
	robot[0]->UpdateMap(world, GetRobotIds());
	// update all robot targetPosition
	for (int i = 0; i < robot.size(); i++)
		for (int j = 0; j < allTargets.size(); j++)
			if (robot[i]->taskID == allTargets[j]->id) {
				robot[i]->targetPosition = allTargets[j]->taskPoint;
				break;
			}
	// path planning for the leaders
	vector<Point> planPath;
	planPath = robot[0]->AStarPath();
	// map the followers 1~robotNumber
	for (int i = 1; i < robotNumber; i++) 
		robot[i]->MappingPath(planPath);
	// see the path
	/*
	for (int i = 0; i < robotNumber; i++) {
		cout << endl << "Path of robot " << robot[i]->id << ": ";
		for (int j = 0; j < robot[i]->planPath.size(); j++)
			cout << "(" << robot[i]->planPath[j].x << " and " << robot[i]->planPath[j].y << "), ";
	}
	*/
}

void 
RobotGroup::TrialMove() {
	// move one attemp step, check collision
	for (int i = 0; i < robotNumber; i++)  robot[i]->TrialStep();
}

void RobotGroup::Move(MatrixMap* world) {
	// update the map
	for (int i = 0; i < robotNumber; i++) 
		world->map_robot(robot[i]->currentPosition.x, robot[i]->currentPosition.y) = 0;
	for (int i = 0; i < robotNumber; i++) 
		world->map_robot(robot[i]->tendPosition.x, robot[i]->tendPosition.y) = robot[i]->id;
	//world->Display();
	// update the robot currentPosition
	for (int i = 0; i < robotNumber; i++) robot[i]->OneStep();

	// delete the first element of planPath
	for (int i = 0; i < robotNumber; i++) {
		if (robot[i]->planPath.size() > 0) {
			vector<Point>::iterator k = robot[i]->planPath.begin();
			robot[i]->planPath.erase(k);
		}
	}
}

vector<Point> 
RobotGroup::GetRobotPos() {
	vector<Point> robotPositions;
	for (int i = 0; i < robotNumber; i++) 
		robotPositions.push_back(robot[i]->tendPosition);
	return robotPositions;
}

vector<int> 
RobotGroup::GetRobotIds() {
	vector<int> robotIds;
	for (int i = 0; i < robotNumber; i++)
		robotIds.push_back(robot[i]->id);
	return robotIds;
}