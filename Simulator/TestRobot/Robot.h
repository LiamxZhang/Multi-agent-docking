#pragma once
// 定义单个机器人的数据格式
// 存储机器人在地图中的所有信息
// weightPoint 定义机器人四邻域的状态
//
#include <iostream>
#include <vector>
#include <Map>

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
	int robotNumber;                 //机器人数量
	int id;                   // robot Id , start from zero
	int taskID;
	int leader;                // leader ID
	Point initPosition;       // initPosition of robot, used for assigning task
	Point currentPosition;    // currentPosition of robot
	Point targetPosition;     // targetPosition of robot

	vector<Point> planPath;   // the planned path of robot, current walking on
	static vector<Point> robotPosition;   //机器人走过的一系列位置
	vector<int> offset;      // position offset to the leader (delta x, delta y)

	int mapRowNumber;         // the row number of map
	int mapColumnNumber;      // the column number of map
	vector<vector<int>> workmap;  // the map robot walks
	vector<vector<WeightPoint>> weightMap;    // the map robot think the map
	

	////////////////////////////////////////////////////
	static vector<float> bidTable;   // 机器人竞价表
	//static mutex mu;                 // 地图锁
	int Status;               // robot status,1 means to go to source of task , 2 means to go to end of task
	bool isBack;              // robot is back to init position
	int waitTime;             // robot waitTime
	float bidPrice;           // the price robot bid for task
	
	int distance;             // 机器人移动距离
	int fightTime;            // 机器人当前最佳移动距离
	int alreadyWait;          // 机器人冲突等待时间
	
	Point nextPosition;       // 机器人之前的位置

	vector<Task> doingTask;   // the task robot is doing
	vector<Task> todoTask;    // the queue of TodoTask
	vector<Task> doneTask;    // the queue of DoneTask
	vector<Task> updateTask;  // 排序结果最好的任务序列

	Robot() :id(0), Status(0), isBack(false), waitTime(-1), bidPrice(0),
		mapRowNumber(0), mapColumnNumber(0), distance(0), fightTime(0), alreadyWait(0),
		initPosition(), nextPosition(), currentPosition(), targetPosition() {
		++robotNumber;
		cout << "Construct Robot Number " << robotNumber << endl;
		//RecordLog("Construct Robot Number " + to_string(robotNumber));
	}
	Robot(int robotnum, int ID, Point position) : robotNumber(robotnum), id(ID), Status(0),
		isBack(false), waitTime(-1), bidPrice(0), mapRowNumber(0), mapColumnNumber(0),
		distance(0), fightTime(0), alreadyWait(0), nextPosition(), currentPosition(), targetPosition() 
	{
		initPosition.x = position.x;
		initPosition.y = position.y;
		currentPosition.x = position.x;
		currentPosition.y = position.y;
		//cout << "Construct Robot Number " << robotNumber << endl;
		//RecordLog("Construct Robot Number " + to_string(robotNumber));
	}
	/*
	bool operator == (const Robot& r) const {
		return (Id == r.Id) && (Status == r.Status) && (isBack == r.isBack) && (waitTime && r.waitTime) &&
			(bidPrice == r.bidPrice) && (mapRowNumber == r.mapRowNumber) &&
			(distance == r.distance) && (fightTime == r.fightTime) &&
			(mapColumnNumber == r.mapColumnNumber) && (initPosition == r.initPosition) &&
			(nextPosition == r.nextPosition) && (alreadyWait == r.alreadyWait) &&
			(currentPosition == r.currentPosition) && (targetPosition == r.targetPosition) &&
			(doingTask == r.doingTask) && (planPath == r.planPath) && (todoTask == r.todoTask) &&
			(doneTask == r.doneTask) && (workmap == r.workmap);
	}
	*/
	// robot get the total robot number, its id and initial position
	bool ReadMap();
	vector<Point> ShortestPath(Point& source, Point& target);   // plan the route of robot
	vector<Point> ShortestPath(Point& source, Point& target, Point Obs);
	bool Move();         // robot move
	bool UpdateMap(MatrixMap*, vector<int>);    // update workmap and weightMap
	bool SaveWeightMap();
	friend int main();
	
	int ShortestDistance(Point& source, Point& target);         // 起点到终点的距离
	void Avoid(Point InputPoint);
	void AvoidClear(Point InputPoint);
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
		cout << " Failed to open the InitMap.txt! " << endl;
		RecordLog("Failed to open the InitMap.txt!");
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
Robot::ShortestPath(Point& source, Point& target) {   // find a route from source to target

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

	vector<Point> result;

	result.push_back(target);
	Point cur = target;
	Point next = relation[cur];
	while (!(relation[cur] == source)) {
		result.push_back(relation[cur]);
		cur = relation[cur];
	}

	result.push_back(source);
	int low = 0, high = result.size() - 1;
	while (low < high) {
		Point tmp = result[low];
		result[low] = result[high];
		result[high] = tmp;
		++low;
		--high;
	}
	return result;
}

vector<Point>
Robot::ShortestPath(Point& source, Point& target, Point Obs) {   //this function is unused

	vector<Point> path;
	vector<vector<int>> tmpworkmap = workmap;
	tmpworkmap[Obs.x][Obs.y] = 1;
	if (tmpworkmap[source.x][source.y] == 1 || tmpworkmap[target.x][target.y] == 1) {
		cout << "起点或终点有障碍物" << endl;
		return path;
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
				next.y < mapColumnNumber && tmpworkmap[next.x][next.y] == 0) {
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

	vector<Point> result;

	result.push_back(target);
	Point cur = target;
	Point next = relation[cur];
	while (!(relation[cur] == source)) {
		result.push_back(relation[cur]);
		cur = relation[cur];
	}

	result.push_back(source);
	int low = 0, high = result.size() - 1;
	while (low < high) {
		Point tmp = result[low];
		result[low] = result[high];
		result[high] = tmp;
		++low;
		--high;
	}
	return result;
}

bool 
Robot::UpdateMap(MatrixMap* world, vector<int> peers) {
	// clear the maps
	workmap.swap(vector<vector<int>>());
	weightMap.swap(vector<vector<WeightPoint>>());
	// update workmap
	for (int r = 0; r < mapRowNumber; ++r) {
		vector<int> oneRow;
		for (int c = 0; c < mapColumnNumber; ++c) {
			bool inPeers = false;
			for (int i = 0; i < peers.size(); i++)
				if (world->map_robot(r, c) == peers[i])
					inPeers = true;
			(world->map_obstacle(r, c) || (world->map_robot(r, c) != 0 && !inPeers))? oneRow.push_back(1) : oneRow.push_back(0);
		}
		workmap.push_back(oneRow);
	}
	// update current weightMap
	string name = "weightMap" + to_string(id);
	ifstream logFile;
	string directory = "../TestRobot/";
	directory = directory + name + ".txt";
	logFile.open(directory, istream::in);
	if (logFile) {
		for (int r = 0; r < mapRowNumber; ++r) {
			for (int c = 0; c < mapColumnNumber; ++c) {
				logFile >> weightMap[r][c].up
					>> weightMap[r][c].down
					>> weightMap[r][c].left
					>> weightMap[r][c].right;
			}
		}
		logFile.close();
	}
	else
		return false;
	return true;
}

bool Robot::SaveWeightMap() {
	//
	string name = "weightMap";
	name = name + to_string(id);
	ofstream logFile;
	string Position = "../TestRobot/";
	Position = Position + name + ".txt";
	logFile.open(Position, ofstream::out);
	if (logFile) {
		for (int r = 0; r < mapRowNumber; ++r) {
			for (int c = 0; c < mapColumnNumber; ++c) {
				logFile << weightMap[r][c].up << " "
					<< weightMap[r][c].down << " "
					<< weightMap[r][c].left << " "
					<< weightMap[r][c].right << endl;
			}
		}
		logFile.close();
		return true;
	}
	else {
		cout << " Failed to open the target file . The target file is Log.txt. " << endl;
		return false;
	}
}