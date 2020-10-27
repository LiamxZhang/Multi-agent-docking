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

#define LogFlag false

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


#pragma region Robot

class Robot {
public:
	int robotNumber;           //robot number
	int id;                    // robot Id , start from zero
	int taskID;
	int leader;                // leader ID
	int step = 0;

	// robot position
	Point initPosition;       // initPosition of robot, used for assigning task
	Point targetPosition;     // targetPosition of robot move
	Point currentPosition;    // currentPosition of robot
	Point tendPosition;       // Position of next one step
	Point lastPosition;       // Position of next one step

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
	Robot(int robotnum, int ID, Point position, int row, int col) : robotNumber(robotnum), id(ID), 
		mapRowNumber(row), mapColumnNumber(col), currentPosition(), targetPosition() {
		initPosition.x = position.x;
		initPosition.y = position.y;
		currentPosition.x = position.x;
		currentPosition.y = position.y;
		lastPosition.x = position.x;
		lastPosition.y = position.y;
		//cout << "Construct Robot Number " << robotNumber << endl;
		//RecordLog("Construct Robot Number " + to_string(robotNumber));
	}
	// robot get the total robot number, its id and initial position
	bool ReadMap(string data_dir);
	vector<Point> AStarPath();   // plan the route of robot, from currentPosition to targetPosition
	vector<Point> AStar(vector<Point> addObstacles);
	vector<Point> MappingPath(vector<Point>);  // mapping the path from leader's path
	void UpdateMap(MatrixMap* world, vector<int> member, vector<int> peers, int interval = 1);    // update workmap and weightMap
	void UpdateLocalMap(MatrixMap* world, vector<int> member, vector<int> peers, int interval = 1);
	void AddtoMap(vector<Point>);
	bool TrialStep();
	void OneStep();         // robot move

	friend int main();
	friend class Rule;
};


bool
Robot::ReadMap(string data_dir) {    // Read the map from given file and form the weight map
	ifstream f;
	f.open(data_dir+"InitMap.txt", ifstream::in);
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
		if (LogFlag)
			RecordLog("Read Robot: Failed to open the InitMap.txt!");
		return false;
	}
	f.close();
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
	int count = 0;
	while (!(relation[cur] == source)) {
		if (count > relation.size()) { // there is no source in relation
			planPath.swap(vector<Point>());
			return planPath;
		}
		planPath.push_back(relation[cur]);
		cur = relation[cur];
		count++;
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

// Astar algorithm
struct Node2D {
	int x;
	int y;
	float g;
	float h;
	float f;
	int idx;
	int preidx;
};

Node2D popNode2D(std::vector<Node2D>& O)
{
	float value = 10000;
	int index = 0;
	for (int i = 0; i < O.size(); i++)
	{
		float f = O[i].g + O[i].h;
		if (f < value) {
			value = f;
			index = i;
		}
	}
	Node2D output = O[index];
	O.erase(O.begin() + index);
	return output;
}

bool AstarAlg(int** map, int height, int width, int startx, int starty, int endx, int endy, vector<int>& pathx, vector<int>& pathy)
{
	//map二维数组，1为障碍物，0为非障碍物
	if (startx < 0 || startx >= width || starty < 0 || starty >= height || endx < 0 || endx >= width || endy < 0 || endy >= height)
		return false;
	
	if (map[startx][starty] == 1 || map[endx][endy] == 1) //起点或终点是障碍物
	//if (map[starty * width + startx] == 1 || map[endy * width + endx] == 1) //起点或终点是障碍物
		return false;

	int directions[4][2] = {{ 0, 1 },{ -1, 0 },{ 1, 0 },{ 0, -1 }}; //四邻域搜索方向
	short* isVisited = new short[height * width];//表示节点状态，0:未访问，1：open,2:close
	for (int i = 0; i < height * width; i++)
		isVisited[i] = 0;

	Node2D* nodes2d = new Node2D[width * height];//存储所有节点

	Node2D startNode, endNode;//初始化起点和终点
	startNode.x = startx;
	startNode.y = starty;
	startNode.g = 0;
	startNode.h = abs(startNode.x - endx) + abs(startNode.y - endy);//使用曼哈顿距离
	startNode.f = startNode.g + startNode.h;
	startNode.idx = starty * width + startx;
	startNode.preidx = -1;//起点没有父节点

	endNode.x = endx;
	endNode.y = endy;


	Node2D nPred;//当前节点
	Node2D nSucc;//子节点
	int iPred, iSucc;//节点id

	vector<Node2D> O;//开列表
	vector<Node2D> C;//闭列表
	O.push_back(startNode);
	isVisited[startNode.idx] = 1;
	nodes2d[startNode.idx] = startNode;

	while (!O.empty())
	{
		nPred = popNode2D(O);
		iPred = nPred.y * width + nPred.x;

		if (nPred.x == endNode.x && nPred.y == endNode.y)
		{
			Node2D tempNode = nPred;
			while (tempNode.preidx > -1)
			{
				pathx.push_back(tempNode.x);
				pathy.push_back(tempNode.y);
				tempNode = nodes2d[tempNode.preidx];
			}
			pathx.push_back(tempNode.x);
			pathy.push_back(tempNode.y);

			int start = 0;
			int end = pathx.size() - 1;
			while (start < end)//反转一下顺序
			{
				swap(pathx[start], pathx[end]);
				swap(pathy[start], pathy[end]);
				start++;
				end--;
			}
			delete[]isVisited;
			delete[]nodes2d;
			return true;
		}

		isVisited[iPred] = 2;//更新为close状态
		C.push_back(nodes2d[iPred]);


		for (char i = 0; i < 4; i++)
		{
			int xSucc = nPred.x + directions[i][0];
			int ySucc = nPred.y + directions[i][1];
			if (xSucc >= 0 && xSucc < width && ySucc >= 0 && ySucc < height && (map[xSucc][ySucc] != 1 && isVisited[ySucc * width + xSucc] != 2))
			//if (xSucc >= 0 && xSucc < width && ySucc >= 0 && ySucc < height && (map[ySucc * width + xSucc] != 1 && isVisited[ySucc * width + xSucc] != 2))
				//边界检测、非障碍物、不在closed链表中
			{
				nSucc.x = xSucc;
				nSucc.y = ySucc;
				nSucc.idx = ySucc * width + xSucc;
				nSucc.preidx = nPred.idx;
				nSucc.g = nPred.g + sqrt((xSucc - nPred.x) * (xSucc - nPred.x) + (ySucc - nPred.y) * (ySucc - nPred.y));
				nSucc.h = abs(nSucc.x - endNode.x) + abs(nSucc.y - endNode.y);
				nSucc.f = nSucc.g + nSucc.h;
				iSucc = nSucc.y * width + nSucc.x;

				if (isVisited[iSucc] == 0)//第一次访问
				{
					isVisited[iSucc] = 1;//更新状态
					nodes2d[iSucc] = nSucc;
					O.push_back(nSucc);//插入Open
				}
				else if (isVisited[iSucc] == 1 && nSucc.f + 0.00001 < nodes2d[iSucc].f)//处于open表中
				{
					nodes2d[iSucc] = nSucc;
					for (int i = 0; i < O.size(); i++) //查找Open表中的该点并替换
					{
						if (O[i].x == nSucc.x && O[i].y == nSucc.y) {
							O[i] = nSucc;//更新Open
							break;
						}
					}
				}
			}
		}

	}
	delete[]isVisited;
	delete[]nodes2d;
	return false;
}

vector<Point> 
Robot::AStar(vector<Point> addObstacles) {
	// update the map
	int** map = new int* [mapRowNumber];
	for (int i = 0; i < mapRowNumber; i++)
		map[i] = new int[mapColumnNumber];

	for (int i = 0; i < mapRowNumber; i++) {
		for (int j = 0; j < mapColumnNumber; j++) {
			if (workmap[i][j]) {
				map[i][j] = 1;
			}
			else
				map[i][j] = 0;
		}
	}
	
	// path planning
	vector<vector<int>> pathVector = { vector<int> (), vector<int> ()};
	bool result = AstarAlg(map, mapRowNumber, mapColumnNumber, currentPosition.x, currentPosition.y, 
		targetPosition.x, targetPosition.y, pathVector[0], pathVector[1]);

	// ouput
	planPath.swap(vector<Point> ());
	if (result == true) {
		for (int i = 0; i < pathVector[0].size(); i++) {
			Point pathPoint(pathVector[0][i], pathVector[1][i]);
			planPath.push_back(pathPoint);
		}
	}
	
	// show the map
	cout << "Robot " << id << "'s work map: " << endl;
	string pathstar;
	for (auto i = 0; i < mapRowNumber; i++) {
		{
			for (auto j = 0; j < mapColumnNumber; j++) {
				pathstar = " ";
				for (int k = 0; k < pathVector[0].size(); k++) {
					if (i == pathVector[0][k] && j == pathVector[1][k]) {
						pathstar = "'";
						break;
					}
				}

				bool add = false;
				for (int k = 0; k < addObstacles.size(); k++) {
					if (i == addObstacles[k].x && j == addObstacles[k].y) {
						cout << "#" << pathstar;
						add = true;
						break;
					}
				}
				if (add) continue;

				if (i == currentPosition.x && j == currentPosition.y) {
					cout << id << pathstar;
				}
				else if (i == targetPosition.x && j == targetPosition.y) {
					cout << "*" << pathstar;
				}
				else {
					cout << map[i][j] << pathstar;
				}
			}
			cout << endl;
		}
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
Robot::UpdateMap(MatrixMap* world, vector<int> member, vector<int> peers, int interval) {
	int obs_value = 1;
	int rob_value = 2;
	// clear the maps // member means in the same group // peer means in the to-be-docked group
	workmap.swap(vector<vector<int>>());
	weightMap.swap(vector<vector<WeightPoint>>());
	// find all robots // deal with members, peers, and others
	MatrixMap* mapForPlan = new MatrixMap(world->RowNum, world->ColNum);
	for (int r = 0; r < world->RowNum; ++r) {
		for (int c = 0; c < world->ColNum; ++c) {
			if (world->map_obstacle(r, c)) // obstacle
				mapForPlan->map_obstacle(r, c) = obs_value;
			else if (world->map_robot(r, c)) { // robots
				int tempRobot = world->map_robot(r, c);
				bool isMember = false;
				for (int i = 0; i < member.size(); ++i) { // members: in the same group
					if (member[i] == tempRobot) {
						isMember = true;
						break;
					}
				}
				if (isMember) continue; // is 0
				// not a member
				bool isPeer = false;
				for (int i = 0; i < peers.size(); ++i) { // peers: to be docked
					if (peers[i] == tempRobot) {
						isPeer = true;
						break;
					}
				}
				if (isPeer) {
					mapForPlan->map_robot(r, c) = rob_value; // small 2
					continue;
				}
				// not a peer
				{ // others should be large and unreachable 2
					int minX, maxX, minY, maxY;
					r - interval > 0 ? minX = r - interval : minX = 0;
					r + interval < mapRowNumber - 1 ? maxX = r + interval : maxX = mapRowNumber - 1;
					c - interval > 0 ? minY = c - interval : minY = 0;
					c + interval < mapColumnNumber - 1 ? maxY = c + interval : maxY = mapColumnNumber - 1;
					for (int i = minX; i <= maxX; ++i) {
						for (int j = minY; j <= maxY; ++j) {
							if (mapForPlan->map_robot(i, j) == 0 && mapForPlan->map_obstacle(i, j) == 0) 
								mapForPlan->map_robot(i, j) = rob_value;
						}
					}
					
				}
			}
			
		}
	}

	// update workmap
	for (int r = 0; r < mapRowNumber; ++r) {
		vector<int> oneRow;
		for (int c = 0; c < mapColumnNumber; ++c) {
			if (mapForPlan->map_obstacle(r, c) == obs_value)  // if obstacle, 1
				oneRow.push_back(obs_value);
			else if (mapForPlan->map_robot(r, c) == rob_value)  // if other robots, 2
				oneRow.push_back(rob_value);
			else
				oneRow.push_back(0);
		}
		workmap.push_back(oneRow);
	}
	//cout << "Weighted map of robot" << id << " for planning: " << endl;
	//mapForPlan->Display("robot");

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

void
Robot::UpdateLocalMap(MatrixMap* world, vector<int> members, vector<int> peers, int interval) {
	int obs_value = 1;
	int rob_value = 2;
	// clear the maps // member means in the same group // peer means in the to-be-docked group
	workmap.swap(vector<vector<int>>());
	weightMap.swap(vector<vector<WeightPoint>>());
	// obstacles
	MatrixMap* mapForPlan = new MatrixMap(world->RowNum, world->ColNum);
	for (int r = 0; r < mapRowNumber; ++r) {
		for (int c = 0; c < mapColumnNumber; ++c) {
			if (world->map_obstacle(r, c) == obs_value) // obstacle
				mapForPlan->map_obstacle(r, c) = obs_value;
		}
	}
	
	// find all robots in neighbors // deal with members, peers, and others
	int minX, maxX, minY, maxY;
	int range = interval + 1; // detect 1 more unit far
	currentPosition.x + range < world->ColNum - 1 ? maxX = currentPosition.x + range : maxX = world->ColNum - 1; // max X
	currentPosition.x - range > 0 ? minX = currentPosition.x - range : minX = 0;                                 // min X
	currentPosition.y + range < world->RowNum - 1 ? maxY = currentPosition.y + range : maxY = world->RowNum - 1; // max Y
	currentPosition.y - range > 0 ? minY = currentPosition.y - range : minY = 0;                                 // min Y

	for (int x = minX; x <= maxX; ++x) {
		for (int y = minY; y <= maxY; ++y) {
			if (world->map_robot(x, y) != 0) { // robots
				int tempRobot = world->map_robot(x, y);
				bool isMember = false;
				for (int i = 0; i < members.size(); ++i) { // members: in the same group
					if (members[i] == tempRobot) {
						isMember = true;
						break;
					}
				}
				if (isMember) continue; // is 0

				// not in members
				bool isPeer = false;
				for (int i = 0; i < peers.size(); ++i) { // peers: to be docked
					if (peers[i] == tempRobot) {
						isPeer = true;
						break;
					}
				}
				if (isPeer) {
					mapForPlan->map_robot(x, y) = rob_value; // small 2
					continue;
				}

				// not in peers
				{ // others should be large and unreachable 2
					int Xmin, Xmax, Ymin, Ymax;
					x - interval > 0 ? Xmin = x - interval : Xmin = 0;
					x + interval < mapRowNumber - 1 ? Xmax = x + interval : Xmax = mapRowNumber - 1;
					y - interval > 0 ? Ymin = y - interval : Ymin = 0;
					y + interval < mapColumnNumber - 1 ? Ymax = y + interval : Ymax = mapColumnNumber - 1;
					/*
					for (int i = Xmin; i <= Xmax; ++i) {
						for (int j = Ymin; j <= Ymax; ++j) {
							if (mapForPlan->map_robot(i, j) == 0 && mapForPlan->map_obstacle(i, j) == 0)
								mapForPlan->map_robot(i, j) = rob_value;
						}
					}
					*/
					//
					for (int i = Xmin; i <= Xmax; ++i) {
						if (mapForPlan->map_robot(i, y) == 0 && mapForPlan->map_obstacle(i, y) == 0)
							mapForPlan->map_robot(i, y) = rob_value;
					}
					for (int j = Ymin; j <= Ymax; ++j) {
						if (mapForPlan->map_robot(x, j) == 0 && mapForPlan->map_obstacle(x, j) == 0)
							mapForPlan->map_robot(x, j) = rob_value;
					}
				}
			}

		}
	}
	// current position
	mapForPlan->map_robot(currentPosition.x, currentPosition.y) = 0;
	// target position
	if (world->map_robot(targetPosition.x, targetPosition.y) == 0) {
		mapForPlan->map_robot(targetPosition.x, targetPosition.y) = 0;
	}

	// update workmap
	for (int r = 0; r < mapRowNumber; ++r) {
		vector<int> oneRow;
		for (int c = 0; c < mapColumnNumber; ++c) {
			if (mapForPlan->map_obstacle(r, c) == obs_value)  // if obstacle, 1
				oneRow.push_back(obs_value);
			else if (mapForPlan->map_robot(r, c) == rob_value)  // if other robots, 2
				oneRow.push_back(rob_value);
			else
				oneRow.push_back(0);
		}
		workmap.push_back(oneRow);
	}
	//cout << "Weighted map of robot" << id << " for planning: " << endl;
	//mapForPlan->Display("robot");
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
				((i - 1) >= 0 && (i - 1) < mapRowNumber) ? t.up = 1 - workmap[i - 1][j] : t.up = -1;
				((i + 1) >= 0 && (i + 1) < mapRowNumber) ? t.down = 1 - workmap[i + 1][j] : t.down = -1;
				((j - 1) >= 0 && (j - 1) < mapColumnNumber) ? t.left = 1 - workmap[i][j - 1] : t.left = -1;
				((j + 1) >= 0 && (j + 1) < mapColumnNumber) ? t.left = 1 - workmap[i][j + 1] : t.right = -1;
			}
			oneRow.push_back(t);
		}
		weightMap.push_back(oneRow);
	}
}

void 
Robot::AddtoMap(vector<Point> addObstacles) {
	for (int i = 0; i < addObstacles.size(); i++) {
		if (addObstacles[i].x != currentPosition.x || addObstacles[i].y != currentPosition.y) {
			workmap[addObstacles[i].x][addObstacles[i].y] = 1;
			weightMap[addObstacles[i].x][addObstacles[i].y].up = -1;
			weightMap[addObstacles[i].x][addObstacles[i].y].down = -1;
			weightMap[addObstacles[i].x][addObstacles[i].y].left = -1;
			weightMap[addObstacles[i].x][addObstacles[i].y].right = -1;
		}
	}
}

bool 
Robot::TrialStep() {   // change tendPosition
	lastPosition.x = currentPosition.x;
	lastPosition.y = currentPosition.y;
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
	step++;

	// delete the first element of planPath
	if (planPath.size() > 0) {
		vector<Point>::iterator k = planPath.begin();
		planPath.erase(k);
	}
}

#pragma endregion

/////////////////////////////////////////////////////////
#pragma region RobotGroup

class RobotGroup {
public:
	int robotNumber;   // robot number in this group
	int leaderID;        // leader robot ID
	int leaderIndex;   // leader robot Index
	vector<Robot*> robot; // group members
	vector<Point> addObstacles; // additional obstacles during moving

	RobotGroup(vector<Robot*> r) : robotNumber(r.size()), robot(r) { AssignLeaders(0); }
	bool AssignLeaders(int);
	void Display() { for (int k = 0; k < robot.size(); k++)   cout << robot[k]->id << ", "; }
	void PathPlanning(MatrixMap* world, vector<TaskPoint*> allTargets, vector<int> peers, int interval = 1);
	void LocalPathPlanning(MatrixMap* world, vector<TaskPoint*> allTargets, vector<int> peers, int newTarget = INT_MAX);
	void LocalPathPlanning(MatrixMap* world, vector<TaskPoint*> allTargets, vector<int> peerIDs, vector<char> segDir_childSide, int newTarget = INT_MAX, int interval = 1);
	void TrialMove();
	void Move(MatrixMap*);
	vector<Point> GetTendPos();
	vector<int> GetRobotIds();
	vector<Point> UpdateMap_Gate(vector<TaskPoint*> allTargets, char segDir = '\0', char childSide = '\0');
};

// assign the leader value (robot ID)
// build the mapping relation inside one component between the leader and the follower 
// leader + offset = robot position
bool 
RobotGroup::AssignLeaders(int ID) {
	if (!robot.size()) return false;
	// find the leader robot ID value
	leaderIndex = ID;
	leaderID = robot[ID]->id;

	// assign
	for (int k = 0; k < robot.size(); k++) {
		// assign the leader value (robot ID)
		robot[k]->leader = robot[ID]->id;
		// build the mapping relation inside one component between the leader and the follower // leader + offset = robot position
		robot[k]->offset.swap(vector<int>());
		robot[k]->offset.push_back(robot[k]->currentPosition.x - robot[ID]->currentPosition.x);
		robot[k]->offset.push_back(robot[k]->currentPosition.y - robot[ID]->currentPosition.y);
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
RobotGroup::PathPlanning(MatrixMap* world, vector<TaskPoint*> allTargets, vector<int> peers, int interval) {
	// leader update robot workmap, weightMap
	robot[leaderIndex]->UpdateMap(world, GetRobotIds(), peers, interval);
	// update all robot targetPosition
	for (int i = 0; i < robot.size(); i++)
		for (int j = 0; j < allTargets.size(); j++)
			if (robot[i]->taskID == allTargets[j]->id) {
				robot[i]->targetPosition = allTargets[j]->taskPoint;
				break;
			}
	
	// path planning for the leaders
	vector<Point> planPath;
	//cout << "Start path planning! " << endl;
	planPath = robot[leaderIndex]->AStarPath();
	
	cout << "planPath: " << endl;
	for (int i = 0; i < planPath.size(); ++i) {
		cout << "(" << planPath[i].x << ", " << planPath[i].y << "), ";
	}
	cout << endl;

	// map the followers 1~robotNumber
	for (int i = 0; i < robotNumber; i++) {
		if (i == leaderIndex) 
			continue;
		robot[i]->MappingPath(planPath);
	}
		
	
	// see the path
	/*
	for (int i = 0; i < robotNumber; i++) {
		cout << endl << "Path of robot " << robot[i]->id << ": ";
		for (int j = 0; j < robot[i]->planPath.size(); j++)
			cout << "(" << robot[i]->planPath[j].x << " and " << robot[i]->planPath[j].y << "), ";
	}
	*/
}

// for Naive algorithm
void
RobotGroup::LocalPathPlanning(MatrixMap* world, vector<TaskPoint*> allTargets, vector<int> peers, int newTarget) {
	// newTarget is the (N-1) point after the current position
	// peers are in the to-be-docked group
	// leader update robot workmap, weightMap
	robot[leaderIndex]->UpdateLocalMap(world, GetRobotIds(), peers);

	// update all robot targetPosition
	for (int i = 0; i < robot.size(); i++)
		for (int j = 0; j < allTargets.size(); j++)
			if (robot[i]->taskID == allTargets[j]->id) {
				robot[i]->targetPosition = allTargets[j]->taskPoint;
				break;
			}
	/*
	for (int i = 0; i < robot.size(); i++) {
		if (newTarget > robot[i]->planPath.size())
			newTarget = robot[i]->planPath.size();

		if (newTarget) {
			robot[i]->targetPosition = robot[i]->planPath[newTarget - 1];
		}
		else {
			for (int j = 0; j < allTargets.size(); j++) {
				if (robot[i]->taskID == allTargets[j]->id) {
					robot[i]->targetPosition = allTargets[j]->taskPoint;
					break;
				}
			}
		}
	}
	*/
	cout << "Start local path planning! " << endl;
	// path planning for the leaders
	vector<Point> oldPath = robot[leaderIndex]->planPath;
	int oldPath_Length = robot[leaderIndex]->planPath.size();
	vector<Point> newPath = robot[leaderIndex]->AStar(addObstacles);
	// append the old path
	for (int i = newTarget; i < oldPath_Length; ++i) {
		robot[leaderIndex]->planPath.push_back(oldPath[i]);
	}

	cout << "local planned Path: " << endl;
	for (int i = 0; i < robot[leaderIndex]->planPath.size(); ++i) {
		cout << "(" << robot[leaderIndex]->planPath[i].x << ", " << robot[leaderIndex]->planPath[i].y << "), ";
	}
	cout << endl;

	// map the followers 1~robotNumber
	for (int i = 0; i < robotNumber; i++) {
		if (i == leaderIndex)
			continue;
		robot[i]->MappingPath(robot[leaderIndex]->planPath);
	}

}

// for common functions
void
RobotGroup::LocalPathPlanning(MatrixMap* world, vector<TaskPoint*> allTargets, vector<int> peerIDs, vector<char> segDir_childSide, int newTarget, int interval) {
	// newTarget is the (N-1) point after the current position
	// peers are in the to-be-docked group

	// leader update robot workmap, weightMap
	robot[leaderIndex]->UpdateLocalMap(world, GetRobotIds(), peerIDs, interval);
	// update the target gate
	/*
	UpdateMap_Gate(allTargets, segDir_childSide[0], segDir_childSide[1]);
	cout << "The added obstacles are: ";
	for (int i = 0; i < addObstacles.size(); ++i) {
		cout << "(" << addObstacles[i].x << ", " << addObstacles[i].y << "), ";
	}
	cout << endl;
	*/
	// update all robot targetPosition
	for (int i = 0; i < robot.size(); i++)
		for (int j = 0; j < allTargets.size(); j++)
			if (robot[i]->taskID == allTargets[j]->id) {
				robot[i]->targetPosition = allTargets[j]->taskPoint;
				break;
			}
	

	/*
	for (int i = 0; i < robot.size(); i++) {
		if (newTarget > robot[i]->planPath.size())
			newTarget = robot[i]->planPath.size();

		if (newTarget) {
			robot[i]->targetPosition = robot[i]->planPath[newTarget - 1];
		}
		else {
			for (int j = 0; j < allTargets.size(); j++) {
				if (robot[i]->taskID == allTargets[j]->id) {
					robot[i]->targetPosition = allTargets[j]->taskPoint;
					break;
				}
			}
		}
	}
	*/
	cout << "Start local path planning! " << endl;
	// path planning for the leaders
	vector<Point> oldPath = robot[leaderIndex]->planPath;
	int oldPath_Length = robot[leaderIndex]->planPath.size();
	vector<Point> newPath = robot[leaderIndex]->AStar(addObstacles);
	// append the old path
	for (int i = newTarget; i < oldPath_Length; ++i) {
		robot[leaderIndex]->planPath.push_back(oldPath[i]);
	}

	cout << "local planned Path: " << endl;
	for (int i = 0; i < robot[leaderIndex]->planPath.size(); ++i) {
		cout << "(" << robot[leaderIndex]->planPath[i].x << ", " << robot[leaderIndex]->planPath[i].y << "), ";
	}
	cout << endl;
	
	// map the followers 1~robotNumber
	for (int i = 0; i < robotNumber; i++) {
		if (i == leaderIndex)
			continue;
		robot[i]->MappingPath(robot[leaderIndex]->planPath);
	}

}


void RobotGroup::TrialMove() {
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
	
}

vector<Point> 
RobotGroup::GetTendPos() {
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

// update the gate positions at the target area
vector<Point>
RobotGroup::UpdateMap_Gate(vector<TaskPoint*> allTargets, char segDir, char childSide) {
	// segDir: x or y // childSide: l or r
	if (segDir == '\0' && childSide == '\0')
		return vector<Point>();

	// get all targets
	vector<Point> targetPositions;
	for (int i = 0; i < robot.size(); i++)
		for (int j = 0; j < allTargets.size(); j++)
			if (robot[i]->taskID == allTargets[j]->id) {
				targetPositions.push_back(allTargets[j]->taskPoint);
				break;
			}

	// find the boundary
	vector<int> boundary = {INT_MIN,INT_MAX,INT_MIN,INT_MAX }; // maxX, minX, maxY, minY
	for (int i = 0; i < targetPositions.size(); ++i) {
		if (targetPositions[i].x > boundary[0]) { boundary[0] = targetPositions[i].x; }
		if (targetPositions[i].x < boundary[1]) { boundary[1] = targetPositions[i].x; }
		if (targetPositions[i].y > boundary[2]) { boundary[2] = targetPositions[i].y; }
		if (targetPositions[i].y < boundary[3]) { boundary[3] = targetPositions[i].y; }
	}

	// determine the target position
	addObstacles.swap(vector<Point>());
	int X1, X2;
	boundary[0] + 1 < robot[0]->mapColumnNumber ? X1 = boundary[0] + 1 : X1 = -1;
	boundary[1] - 1 > 0 ? X2 = boundary[1] - 1 : X2 = -1;
	int Y1, Y2;
	boundary[2] + 1 < robot[0]->mapRowNumber ? Y1 = boundary[2] + 1 : Y1 = -1;
	boundary[3] - 1 > 0 ? Y2 = boundary[3] - 1 : Y2 = -1;
	
	if (segDir == 'x' && childSide == 'l') {
		// left, minX
		if (Y1 >= 0) {
			Point add1(boundary[1], Y1);
			addObstacles.push_back(add1);
		}
		if (Y2 >= 0) {
			Point add2(boundary[1], Y2);
			addObstacles.push_back(add2);
		}
	}
	else if (segDir == 'x' && childSide == 'r') {
		// right, maxX
		if (Y1 >= 0) {
			Point add1(boundary[0], Y1);
			addObstacles.push_back(add1);
		}
		if (Y2 >= 0) {
			Point add2(boundary[0], Y2);
			addObstacles.push_back(add2);
		}
	}
	else if (segDir == 'y' && childSide == 'l') {
		// up, minY
		if (X1 >= 0) {
			Point add1(X1, boundary[3]);
			addObstacles.push_back(add1);
		}
		if (X2 >= 0) {
			Point add2(X2, boundary[3]);
			addObstacles.push_back(add2);
		}
	}
	else if (segDir == 'y' && childSide == 'r') {
		// down, maxY
		if (X1 >= 0) {
			Point add1(X1, boundary[2]);
			addObstacles.push_back(add1);
		}
		if (X2 >= 0) {
			Point add2(X2, boundary[2]);
			addObstacles.push_back(add2);
		}
	}
	
	// update the map with gate
	robot[leaderIndex]->AddtoMap(addObstacles);

	return addObstacles;
}

#pragma endregion