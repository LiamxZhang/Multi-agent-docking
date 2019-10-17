#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include <math.h>
#include <algorithm>
#include <map>
#include <windows.h>
using namespace std;
//地图点类的定义
class Point {
public:
	Point() :x(0), y(0), Assignedflag(false) {}
	Point(int tx, int ty) {
		x = tx;
		y = ty;
	}
	bool operator==(const Point& r) const {
		return (x == r.x) && (y == r.y);
	}
	bool operator < (const Point& r) const {
		int a = x * x + y * y;
		int b = r.x * r.x + r.y * r.y;
		if (a < b) {
			return true;
		}
		else if (a == b) {
			if (x < r.x)
				return true;
			else
				return false;
		}
		else {
			return false;
		}
	}
	Point up() {
		return Point(x - 1, y);
	}
	Point down() {
		return Point(x + 1, y);
	}
	Point left() {
		return Point(x, y - 1);
	}
	Point right() {
		return Point(x, y + 1);
	}
	int id;
	int x;
	int y;
	bool Assignedflag, Leaderflag = true;
	bool SetSearched = false;
	bool Dockedflag = false;
	bool OffsetFound = false;
	bool LeftFlag = false;//默认为右
	int LeaderId, RobotId;
	int status = 1;
private:
};
//权重点类的定义
class WeightPoint {
private:
public:
	int up;
	int down;
	int left;
	int right;
	WeightPoint() :up(0), down(0), left(0), right(0) {}
	bool operator == (const WeightPoint& r) const {
		return (up == r.up) && (down == r.down) && (left == r.left) && (right == r.right);
	}
};
//每个对接体的偏移点集
struct SET
{
	//包括机器人Id和横纵坐标的偏移量
	int RobotId, Offsetx, Offsety;
};
//所有对接体的偏移点集
struct OffsetStruct
{
	//存放每个对接体的Id和单个对接体点集
	int SetId;
	vector<SET> Set;
};
int mapRowNumber, mapColumnNumber, robotNumber;
int num = 0;
bool VerFirst = true;
vector<OffsetStruct> Offset;
vector<vector<vector<WeightPoint>>> weightMap;
vector<vector<int>> Workmap, Copymap;
vector<vector<vector<float>>> ProbMap;
vector<vector<vector<bool>>> KnownMap;
vector<Point> robotInitPosition;
vector<Point> robotTargetPosition;
vector<Point> robotCurrentPosition;
vector<Point> robotPreviousPosition;
vector<Point> SingleTarget;
int linenum;
int Stucknum = 0;
void TaskAssign() {//任务分配
	//srand((unsigned)time(NULL));
	vector<int> aset;
	for (int i = 0; i < INT_MAX; ++i) {
		int a = rand() % (8);
		int testnum = 0;
		for (int j = 0; j < aset.size(); ++j) {
			if (a == aset[j]) {
				break;
			}
			else {
				++testnum;
				continue;
			}
		}
		if (testnum == aset.size()) {
			aset.push_back(a);
		}
		if (aset.size() == 8) {
			break;
		}
	}//Randomly assign task
	for (int i = 0; i < robotNumber; ++i) {
		int Smallest = INT_MAX, SmallId = -1, Diff;
		for (int j = 0; j < robotNumber; ++j) {
			if (robotInitPosition[j].Assignedflag == false) {
				Diff = abs(robotTargetPosition[i].x - robotInitPosition[j].x) +
					abs(robotTargetPosition[i].y - robotInitPosition[j].y);
				if (Smallest > Diff) {
					Smallest = Diff;
					SmallId = j;
				}//This part is used to assign tasks according to distance
				//SmallId = aset.front();	
			}
		}
		robotTargetPosition[i].id = SmallId;
		aset.erase(aset.begin());
		robotInitPosition[SmallId].Assignedflag = true;
	}
	for (int i = 1; i < robotNumber; ++i) {//Sort the assigned tasks
		int factor = 1;
		while (i - factor >= 0) {
			if (robotTargetPosition[i - factor].id > robotTargetPosition[i - factor + 1].id) {
				swap(robotTargetPosition[i - factor].id, robotTargetPosition[i - factor + 1].id);
				swap(robotTargetPosition[i - factor].x, robotTargetPosition[i - factor + 1].x);
				swap(robotTargetPosition[i - factor].y, robotTargetPosition[i - factor + 1].y);
			}
			++factor;
		}
	}
}
void UpdateWeightMap() {//更新权重地图
	for (int a = 0; a < robotNumber; ++a) {
		vector<vector<WeightPoint>> SingleweightMap;
		for (int i = 0; i < mapRowNumber; ++i) {
			vector<WeightPoint> oneRow;
			for (int j = 0; j < mapColumnNumber; ++j) {
				WeightPoint t;
				if (ProbMap[a][i][j] != 0) {
					t.up = -1;
					t.down = -1;
					t.left = -1;
					t.right = -1;
				}
				/*else if ((i - 1) >= 0 && (i - 1) < mapRowNumber) {
					if(ProbMap[a][i-1][j] != 0)
					t.up = -1;
					t.down = -1;
					t.left = -1;
					t.right = -1;
				}*/
				else {
					if ((i - 1) >= 0 && (i - 1) < mapRowNumber) {
						t.up = 1 - ProbMap[a][i - 1][j];
					}
					else
						t.up = -1;
					if ((i + 1) >= 0 && (i + 1) < mapRowNumber) {
						t.down = 1 - ProbMap[a][i + 1][j];
					}
					else
						t.down = -1;
					if ((j - 1) >= 0 && (j - 1) < mapColumnNumber) {
						t.left = 1 - ProbMap[a][i][j - 1];
					}
					else
						t.left = -1;
					if ((j + 1) >= 0 && (j + 1) < mapColumnNumber) {
						t.left = 1 - ProbMap[a][i][j + 1];
					}
					else
						t.right = -1;
				}
				oneRow.push_back(t);
			}
			SingleweightMap.push_back(oneRow);
		}
		weightMap.push_back(SingleweightMap);
	}
}
void ReadMap() {//读取地图
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
					f >> t;
			}
			Workmap.push_back(oneRow);
			Copymap.push_back(oneRow);
		}
	}
	for (int h = 0; h < robotNumber; ++h) {//Initial Each Probability Map, 0 means free, 1 means occupied
		vector<vector<float>> onematrix;
		for (int i = 0; i < mapRowNumber; ++i) {
			vector<float> oneRow;
			for (int j = 0; j < mapColumnNumber; ++j) {
				oneRow.push_back(0);
			}
			onematrix.push_back(oneRow);
		}
		ProbMap.push_back(onematrix);
	}
	for (int h = 0; h < robotNumber; ++h) {//Initial Each Probability Map, 0 means free, 1 means occupied
		vector<vector<bool>> onematrix;
		for (int i = 0; i < mapRowNumber; ++i) {
			vector<bool> oneRow;
			for (int j = 0; j < mapColumnNumber; ++j) {
				oneRow.push_back(false);
			}
			onematrix.push_back(oneRow);
		}
		KnownMap.push_back(onematrix);
	}
	UpdateWeightMap();
	f.close();
}
void Dock() {
	for (int i = 0; i < robotNumber; ++i) {
		if (robotCurrentPosition[i].Dockedflag == false) {
			int x = robotCurrentPosition[i].x, y = robotCurrentPosition[i].y;
			if ((x - 1) >= 0 && (x - 1) < mapRowNumber && Workmap[x - 1][y] == 2 ||
				(x + 1) >= 0 && (x + 1) < mapRowNumber && Workmap[x + 1][y] == 2 ||
				(y - 1) >= 0 && (y - 1) < mapColumnNumber && Workmap[x][y - 1] == 2 ||
				(y + 1) >= 0 && (y + 1) < mapColumnNumber && Workmap[x][y + 1] == 2) {
				robotCurrentPosition[i].Dockedflag = true;
			}
		}
	}//更新机器人的合并标志
}
void ChangeMap() {//更新地图中机器人的位置，即动态障碍
	for (int i = 0; i < mapRowNumber; ++i) {
		for (int j = 0; j < mapColumnNumber; ++j) {
			Workmap[i][j] = Copymap[i][j];
		}
	}
	for (int i = 0; i < robotNumber; ++i) {
		if (KnownMap[i][robotCurrentPosition[i].x][robotCurrentPosition[i].y] == false) {
			KnownMap[i][robotCurrentPosition[i].x][robotCurrentPosition[i].y] = true;
			ProbMap[i][robotCurrentPosition[i].x][robotCurrentPosition[i].y] = 0;
		}
		Workmap[robotCurrentPosition[i].x][robotCurrentPosition[i].y] = 2;
		/*int x = robotCurrentPosition[i].x, y = robotCurrentPosition[i].y;
		if ((x - 1) >= 0 && (x - 1) < mapRowNumber) {
			Workmap[x - 1][y] = 2;
			if ((y - 1) >= 0 && (y - 1) < mapColumnNumber) {
				Workmap[x - 1][y - 1] = 2;
			}
			if ((y + 1) >= 0 && (y + 1) < mapColumnNumber) {
				Workmap[x - 1][y + 1] = 2;
			}
		}
		if ((x + 1) >= 0 && (x + 1) < mapRowNumber) {
			Workmap[x + 1][y] = 2;
			if ((y - 1) >= 0 && (y - 1) < mapColumnNumber) {
				Workmap[x + 1][y - 1] = 2;
			}
			if ((y + 1) >= 0 && (y + 1) < mapColumnNumber) {
				Workmap[x + 1][y + 1] = 2;
			}
		}
		if ((y - 1) >= 0 && (y - 1) < mapColumnNumber) {
			Workmap[x][y - 1] = 2;
		}
		if ((y + 1) >= 0 && (y + 1) < mapColumnNumber) {
			Workmap[x][y + 1] = 2;
		}*/
	}
	UpdateWeightMap();
	Dock();
}
vector<Point> AStarPlan(int& robotId, Point& source, Point& target) {//A*路径规划
	vector<Point> path;
	if (source.x == target.x && source.y == target.y) {
		path.push_back(source);
		return path;
	}
	struct APoint {
		Point p;
		double g;
		double f;
		bool operator < (const APoint& a) const {
			return f < a.f;
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
				gn = weightMap[robotId][ap.p.x][ap.p.y].up;
			}
			else if (direct == 2) {
				next = cur.right();
				gn = weightMap[robotId][ap.p.x][ap.p.y].right;
			}
			else if (direct == 3) {
				next = cur.down();
				gn = weightMap[robotId][ap.p.x][ap.p.y].down;
			}
			else if (direct == 4) {
				next = cur.left();
				gn = weightMap[robotId][ap.p.x][ap.p.y].left;
			}
			if (next.x >= 0 && next.x < mapRowNumber && next.y >= 0 &&
				next.y < mapColumnNumber && Workmap[next.x][next.y] == 0) {
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
					if (oi == -1) {
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
					else {
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
void ReadRobotInitPosition() {//读取机器人初始位置
	ifstream f;
	f.open("../TestRobot/Robot_Init_Position.txt", ifstream::in);
	if (f) {
		f >> robotNumber;
		for (int i = 0; i < robotNumber; ++i) {
			int x, y;
			int tempdata;
			char tempchar;
			f >> tempdata;
			f >> tempchar;
			f >> tempdata;
			x = tempdata - 1;
			f >> tempchar;
			f >> tempdata;
			y = tempdata - 1;
			Point t;
			t.x = x;
			t.y = y;
			robotInitPosition.push_back(t);
		}
		robotCurrentPosition = robotInitPosition;  // the init of robotCurrentPosition is robotInitPosition
		robotPreviousPosition = robotInitPosition;
	}
	f.close();
	for (int i = 0; i < robotNumber; ++i) {
		robotCurrentPosition[i].LeaderId = i;
		robotCurrentPosition[i].RobotId = i;//This will never change
	}
}
void ReadTask() {//读取任务文件
	ifstream f;
	f.open("../TestRobot/Task.txt", ifstream::in);
	if (f) {
		Point tempTask;
		for (int i = 0; i < robotNumber; ++i) {
			f >> tempTask.id >> tempTask.x >> tempTask.y;
			--tempTask.x;
			--tempTask.y;
			robotTargetPosition.push_back(tempTask);
		}
	}
	f.close();
}
void RecordCurrentAndTargetPosition() {//将机器人当前位置写入文件，更新可视化界面
	ofstream f;
	f.open("../TestRobot/Robot_Current_Position.txt", ofstream::out);
	if (f) {
		f << robotCurrentPosition.size() << endl;
		for (int i = 1; i <= robotCurrentPosition.size(); ++i) {
			f << i << "," << robotCurrentPosition[i - 1].x + 1 << "," << robotCurrentPosition[i - 1].y + 1 << ","
				<< robotTargetPosition[i - 1].x + 1 << "," << robotTargetPosition[i - 1].y + 1 << endl;
		}
	}
	f.close();
}
void Perception(int& RobotId) {//机器人感知环境
	int x = robotCurrentPosition[RobotId].x, y = robotCurrentPosition[RobotId].y;
	if ((x - 1) >= 0 && (x - 1) < mapRowNumber) {
		if (KnownMap[RobotId][x - 1][y] == false) {
			if (Workmap[x - 1][y] == 0) {
				KnownMap[RobotId][x - 1][y] = true;
				ProbMap[RobotId][x - 1][y] = 0;
			}
			else {
				ProbMap[RobotId][x - 1][y] = (ProbMap[RobotId][x - 1][y] + 1) / 2;
			}
		}
	}
	if ((x + 1) >= 0 && (x + 1) < mapRowNumber) {
		if (KnownMap[RobotId][x + 1][y] == false) {
			if (Workmap[x + 1][y] == 0) {
				KnownMap[RobotId][x + 1][y] = true;
				ProbMap[RobotId][x + 1][y] = 0;
			}
			else {
				ProbMap[RobotId][x + 1][y] = (ProbMap[RobotId][x + 1][y] + 1) / 2;
			}
		}
	}
	if ((y - 1) >= 0 && (y - 1) < mapColumnNumber) {
		if (KnownMap[RobotId][x][y - 1] == false) {
			if (Workmap[x][y - 1] == 0) {
				KnownMap[RobotId][x][y - 1] = true;
				ProbMap[RobotId][x][y - 1] = 0;
			}
			else {
				ProbMap[RobotId][x][y - 1] = (ProbMap[RobotId][x][y - 1] + 1) / 2;
			}
		}
	}
	if ((y + 1) >= 0 && (y + 1) < mapColumnNumber) {
		if (KnownMap[RobotId][x][y + 1] == false) {
			if (Workmap[x][y + 1] == 0) {
				KnownMap[RobotId][x][y + 1] = true;
				ProbMap[RobotId][x][y + 1] = 0;
			}
			else {
				ProbMap[RobotId][x][y + 1] = (ProbMap[RobotId][x][y + 1] + 1) / 2;
			}
		}
	}

	/*if ((x - 2) >= 0 && (x - 2) < mapRowNumber) {
		if (KnownMap[RobotId][x - 2][y] == false) {
			if (Workmap[x - 2][y] == 0) {
				KnownMap[RobotId][x - 2][y] = true;
				ProbMap[RobotId][x - 2][y] = 0;
			}
			else {
				ProbMap[RobotId][x - 2][y] = (ProbMap[RobotId][x - 2][y] + 1) / 2;
			}
		}
	}
	if ((x + 2) >= 0 && (x + 2) < mapRowNumber) {
		if (KnownMap[RobotId][x + 2][y] == false) {
			if (Workmap[x + 2][y] == 0) {
				KnownMap[RobotId][x + 2][y] = true;
				ProbMap[RobotId][x + 2][y] = 0;
			}
			else {
				ProbMap[RobotId][x + 2][y] = (ProbMap[RobotId][x + 2][y] + 1) / 2;
			}
		}
	}
	if ((y - 2) >= 0 && (y - 2) < mapColumnNumber) {
		if (KnownMap[RobotId][x][y - 2] == false) {
			if (Workmap[x][y - 2] == 0) {
				KnownMap[RobotId][x][y - 2] = true;
				ProbMap[RobotId][x][y - 2] = 0;
			}
			else {
				ProbMap[RobotId][x][y - 2] = (ProbMap[RobotId][x][y - 2] + 1) / 2;
			}
		}
	}
	if ((y + 2) >= 0 && (y + 2) < mapColumnNumber) {
		if (KnownMap[RobotId][x][y + 2] == false) {
			if (Workmap[x][y + 2] == 0) {
				KnownMap[RobotId][x][y + 2] = true;
				ProbMap[RobotId][x][y + 2] = 0;
			}
			else {
				ProbMap[RobotId][x][y + 2] = (ProbMap[RobotId][x][y + 2] + 1) / 2;
			}
		}
	}

	if ((x - 1) >= 0 && (x - 1) < mapRowNumber && (y - 1) >= 0 && (y - 1) < mapColumnNumber) {
		if (KnownMap[RobotId][x - 1][y - 1] == false) {
			if (Workmap[x - 1][y - 1] == 0) {
				KnownMap[RobotId][x - 1][y - 1] = true;
				ProbMap[RobotId][x - 1][y - 1] = 0;
			}
			else {
				ProbMap[RobotId][x - 1][y - 1] = (ProbMap[RobotId][x - 1][y - 1] + 1) / 2;
			}
		}
	}
	if ((x + 1) >= 0 && (x + 1) < mapRowNumber && (y - 1) >= 0 && (y - 1) < mapColumnNumber) {
		if (KnownMap[RobotId][x + 1][y - 1] == false) {
			if (Workmap[x + 1][y - 1] == 0) {
				KnownMap[RobotId][x + 1][y - 1] = true;
				ProbMap[RobotId][x + 1][y - 1] = 0;
			}
			else {
				ProbMap[RobotId][x + 1][y - 1] = (ProbMap[RobotId][x + 1][y - 1] + 1) / 2;
			}
		}
	}
	if ((x - 1) >= 0 && (x - 1) < mapRowNumber && (y + 1) >= 0 && (y + 1) < mapColumnNumber) {
		if (KnownMap[RobotId][x - 1][y + 1] == false) {
			if (Workmap[x - 1][y + 1] == 0) {
				KnownMap[RobotId][x - 1][y + 1] = true;
				ProbMap[RobotId][x - 1][y + 1] = 0;
			}
			else {
				ProbMap[RobotId][x - 1][y + 1] = (ProbMap[RobotId][x - 1][y + 1] + 1) / 2;
			}
		}
	}
	if ((x + 1) >= 0 && (x + 1) < mapRowNumber && (y + 1) >= 0 && (y + 1) < mapColumnNumber) {
		if (KnownMap[RobotId][x + 1][y + 1] == false) {
			if (Workmap[x + 1][y + 1] == 0) {
				KnownMap[RobotId][x + 1][y + 1] = true;
				ProbMap[RobotId][x + 1][y + 1] = 0;
			}
			else {
				ProbMap[RobotId][x + 1][y + 1] = (ProbMap[RobotId][x + 1][y + 1] + 1) / 2;
			}
		}
	}*/
}
void PrintMap(int inputId) {//打印地图
	std::cout << "Iteration " << num << " ..." << endl;
	for (int i = 0; i < mapRowNumber; ++i) {
		for (int j = 0; j < mapColumnNumber; ++j) {
			std::cout << KnownMap[inputId][i][j] << " ";
		}
		std::cout << "| ";
		for (int j = 0; j < mapColumnNumber; ++j) {
			std::cout << ProbMap[inputId][i][j] << " ";
		}
		std::cout << endl;
	}
	++num;
}
void Awayfrom() {
	for (int i = 0; i < robotNumber; ++i) {
		int x = robotCurrentPosition[i].x, y = robotCurrentPosition[i].y;
		for (int j = 1; j < robotNumber - i; ++j) {
			if ((y - 1) >= 0 && (y - 1) < mapColumnNumber) {
				if (robotCurrentPosition[j].x == x &&
					robotCurrentPosition[j].y == y - 1) {
					robotCurrentPosition[i].x = x;
					robotCurrentPosition[i].y = y + 1;//+= 1;
				}
			}
			else if ((y + 1) >= 0 && (y + 1) < mapColumnNumber) {
				if (robotCurrentPosition[j].x == x &&
					robotCurrentPosition[j].y == y + 1) {
					robotCurrentPosition[i].x = x;
					robotCurrentPosition[i].y = y - 1;//+= 1;
				}
			}
		}
		/*if ((x - 1) >= 0 && (x - 1) < mapRowNumber) {
			if (Workmap[x - 1][y] == 2) {
				robotCurrentPosition[i].x += 1;
			}
		}
		else if ((x + 1) >= 0 && (x + 1) < mapRowNumber) {
			if (Workmap[x + 1][y] == 2) {
				robotCurrentPosition[i].x -= 1;
			}
		}*/
		//if ((y - 1) >= 0 && (y - 1) < mapColumnNumber) {
		//	if (Workmap[x][y - 1] == 2) {
		//		robotCurrentPosition[i].x = robotCurrentPosition[i].x;
		//		robotCurrentPosition[i].y = robotCurrentPosition[i].y;//+= 1;
		//	}
		//}
		//else if ((y + 1) >= 0 && (y + 1) < mapColumnNumber) {
		//	if (Workmap[x][y + 1] == 2) {
		//		robotCurrentPosition[i].x = robotCurrentPosition[i].x;
		//		robotCurrentPosition[i].y = robotCurrentPosition[i].y;//-= 1;
		//	}
		//}

		//if ((x - 1) >= 0 && (x - 1) < mapRowNumber && (y - 1) >= 0 && (y - 1) < mapColumnNumber) {
		//	if (Workmap[x - 1][y - 1] == 2) {
		//		//robotCurrentPosition[i].x += 1;
		//		robotCurrentPosition[i].y += 1;
		//	}
		//}
		//if ((x - 1) >= 0 && (x - 1) < mapRowNumber && (y + 1) >= 0 && (y + 1) < mapColumnNumber) {
		//	if (Workmap[x - 1][y + 1] == 2) {
		//		//robotCurrentPosition[i].x += 1;
		//		robotCurrentPosition[i].y -= 1;
		//	}
		//}
		//if ((x + 1) >= 0 && (x + 1) < mapRowNumber && (y - 1) >= 0 && (y - 1) < mapColumnNumber) {
		//	if (Workmap[x + 1][y - 1] == 2) {
		//		//robotCurrentPosition[i].x -= 1;
		//		robotCurrentPosition[i].y += 1;
		//	}
		//}
		//if ((x + 1) >= 0 && (x + 1) < mapRowNumber && (y + 1) >= 0 && (y + 1) < mapColumnNumber) {
		//	if (Workmap[x + 1][y + 1] == 2) {
		//		//robotCurrentPosition[i].x -= 1;
		//		robotCurrentPosition[i].y -= 1;
		//	}
		//}
	}
}
void CollisionAvoid() {//机器人冲突避免
	for (int i = 0; i < robotNumber; ++i) {
		for (int j = 1; j < robotNumber - i; ++j) {
			if (robotCurrentPosition[i].x == robotCurrentPosition[i + j].x &&
				robotCurrentPosition[i].y == robotCurrentPosition[i + j].y) {
				robotCurrentPosition[i] = robotPreviousPosition[i];
				break;
			}
		}
	}
}
void FindLeader() {//在合并团体中搜索leader
	int Leader = INT_MAX;
	for (int i = robotNumber - 1; i >= 0; --i) {
		if (robotCurrentPosition[i].Leaderflag == true &&
			robotCurrentPosition[i].LeaderId == -1 && Leader > robotCurrentPosition[i].RobotId) {
			Leader = robotCurrentPosition[i].RobotId;
			robotCurrentPosition[i].Leaderflag = false;
		}
	}
	for (int i = 0; i < robotNumber; ++i) {
		if (robotCurrentPosition[i].RobotId == Leader) {
			robotCurrentPosition[i].Leaderflag = true;
			break;
		}
	}
	for (int i = 0; i < robotNumber; ++i) {
		if (robotCurrentPosition[i].LeaderId == -1) {
			robotCurrentPosition[i].LeaderId = Leader;
		}
	}//Up to here one leader has been found
	//find offset
	for (int i = 0; i < robotNumber; ++i) {
		if (robotCurrentPosition[i].Dockedflag == true && robotCurrentPosition[i].Leaderflag == true) {//Find the leader of the set
			int x, y;
			x = robotCurrentPosition[i].x;
			y = robotCurrentPosition[i].y;//base coordination
			int LeaderIs = robotCurrentPosition[i].RobotId;
			OffsetStruct tmpOffset;
			tmpOffset.SetId = LeaderIs;//Same SetId means in the same set 
			for (int j = 0; j < robotNumber; ++j) {
				if (robotCurrentPosition[j].Leaderflag == false && robotCurrentPosition[j].LeaderId == LeaderIs &&
					robotCurrentPosition[j].OffsetFound == false) {
					SET tmpSET;
					tmpSET.RobotId = robotCurrentPosition[j].RobotId;//Different RobotId means different robot in the set
					tmpSET.Offsetx = robotCurrentPosition[j].x - x;
					tmpSET.Offsety = robotCurrentPosition[j].y - y;//Offset of each robot in the set
					tmpOffset.Set.push_back(tmpSET);
					robotCurrentPosition[j].OffsetFound = true;
				}
			}
			Offset.push_back(tmpOffset);
		}
	}
}
void SetSearch(int input) {//搜索对接团体
	bool Searched = false;
	int sign;
	for (int i = 0; i < robotNumber; ++i) {
		if (robotCurrentPosition[i].LeaderId == -1) {
			Searched = true;
			sign = i;
			break;
		}
	}
	if (Searched == true) {
		int x = robotCurrentPosition[sign].x, y = robotCurrentPosition[sign].y;
		if ((x - 1) >= 0 && (x - 1) < mapRowNumber && Workmap[x - 1][y] == 2) {
			for (int j = 0; j < robotNumber; ++j) {
				if (robotCurrentPosition[j].LeaderId != -1 &&
					robotCurrentPosition[j].x == x - 1 && robotCurrentPosition[j].y == y) {
					robotCurrentPosition[j].LeaderId = -1;
					robotCurrentPosition[j].SetSearched = true;
					SetSearch(1);
				}
			}
		}
		if ((x + 1) >= 0 && (x + 1) < mapRowNumber && Workmap[x + 1][y] == 2) {
			for (int j = 0; j < robotNumber; ++j) {
				if (robotCurrentPosition[j].LeaderId != -1 &&
					robotCurrentPosition[j].x == x + 1 && robotCurrentPosition[j].y == y) {
					robotCurrentPosition[j].LeaderId = -1;
					robotCurrentPosition[j].SetSearched = true;
					SetSearch(1);
				}
			}
		}
		if ((y - 1) >= 0 && (y - 1) < mapColumnNumber && Workmap[x][y - 1] == 2) {
			for (int j = 0; j < robotNumber; ++j) {
				if (robotCurrentPosition[j].LeaderId != -1 &&
					robotCurrentPosition[j].x == x && robotCurrentPosition[j].y == y - 1) {
					robotCurrentPosition[j].LeaderId = -1;
					robotCurrentPosition[j].SetSearched = true;
					SetSearch(1);
				}
			}
		}
		if ((y + 1) >= 0 && (y + 1) < mapColumnNumber && Workmap[x][y + 1] == 2) {
			for (int j = 0; j < robotNumber; ++j) {
				if (robotCurrentPosition[j].LeaderId != -1 &&
					robotCurrentPosition[j].x == x && robotCurrentPosition[j].y == y + 1) {
					robotCurrentPosition[j].LeaderId = -1;
					robotCurrentPosition[j].SetSearched = true;
					SetSearch(1);
				}
			}
		}
	}
	else {
		sign = input;
		robotCurrentPosition[sign].LeaderId = -1;
		int x = robotCurrentPosition[sign].x, y = robotCurrentPosition[sign].y;
		if ((x - 1) >= 0 && (x - 1) < mapRowNumber && Workmap[x - 1][y] == 2) {
			for (int j = 0; j < robotNumber; ++j) {
				if (robotCurrentPosition[j].LeaderId != -1 &&
					robotCurrentPosition[j].x == x - 1 && robotCurrentPosition[j].y == y) {
					robotCurrentPosition[j].LeaderId = -1;
					robotCurrentPosition[j].SetSearched = true;
					SetSearch(1);
				}
			}
		}
		if ((x + 1) >= 0 && (x + 1) < mapRowNumber && Workmap[x + 1][y] == 2) {
			for (int j = 0; j < robotNumber; ++j) {
				if (robotCurrentPosition[j].LeaderId != -1 &&
					robotCurrentPosition[j].x == x + 1 && robotCurrentPosition[j].y == y) {
					robotCurrentPosition[j].LeaderId = -1;
					robotCurrentPosition[j].SetSearched = true;
					SetSearch(1);
				}
			}
		}
		if ((y - 1) >= 0 && (y - 1) < mapColumnNumber && Workmap[x][y - 1] == 2) {
			for (int j = 0; j < robotNumber; ++j) {
				if (robotCurrentPosition[j].LeaderId != -1 &&
					robotCurrentPosition[j].x == x && robotCurrentPosition[j].y == y - 1) {
					robotCurrentPosition[j].LeaderId = -1;
					robotCurrentPosition[j].SetSearched = true;
					SetSearch(1);
				}
			}
		}
		if ((y + 1) >= 0 && (y + 1) < mapColumnNumber && Workmap[x][y + 1] == 2) {
			for (int j = 0; j < robotNumber; ++j) {
				if (robotCurrentPosition[j].LeaderId != -1 &&
					robotCurrentPosition[j].x == x && robotCurrentPosition[j].y == y + 1) {
					robotCurrentPosition[j].LeaderId = -1;
					robotCurrentPosition[j].SetSearched = true;
					SetSearch(1);
				}
			}
		}
	}//Up to here one set has been found
}
void FollowerMove(int& inputId) {
	int tmpxoffset, tmpyoffset, leaderX, leaderY;
	for (int i = 0; i < Offset.size(); ++i) {
		if (Offset[i].SetId == robotCurrentPosition[inputId].LeaderId) {
			for (int j = 0; j < Offset[i].Set.size(); ++j) {
				if (Offset[i].Set[j].RobotId == robotCurrentPosition[inputId].RobotId) {
					tmpxoffset = Offset[i].Set[j].Offsetx;
					tmpyoffset = Offset[i].Set[j].Offsety;
					for (int i = 0; i < robotNumber; ++i) {
						if (robotCurrentPosition[i].RobotId == robotCurrentPosition[inputId].LeaderId) {
							leaderX = robotCurrentPosition[i].x;
							leaderY = robotCurrentPosition[i].y;
							/*if (robotCurrentPosition[inputId].x == leaderX + tmpxoffset &&
								robotCurrentPosition[inputId].y == leaderY + tmpyoffset) {
								++Stucknum;
								cout << Stucknum << endl;
							}
							else {*/
							robotCurrentPosition[inputId].x = leaderX + tmpxoffset;
							robotCurrentPosition[inputId].y = leaderY + tmpyoffset;
							break;
							//}
						}
					}
					break;
				}
			}
		}
	}
}
void InterLineSegment() {
	linenum = 1 + abs(robotTargetPosition[0].x - robotTargetPosition[robotTargetPosition.size() - 1].x);
	int readline;
	int standard;
	//Interline Segmentation
	if (linenum % 2 == 0) {//line number is even
		readline = 0;
		for (int i = 1; i < robotTargetPosition.size(); ++i) {
			if (robotTargetPosition[i - 1].x == robotTargetPosition[i].x) {
				robotTargetPosition[i - 1].x += 2;
			}
			else {
				robotTargetPosition[i - 1].x += 2;
				++readline;
				if (readline < linenum / 2) {
					continue;
				}
				else {
					break;
				}
			}
		}
		readline = 0;
		for (int i = robotTargetPosition.size() - 1; i >= 0; --i) {
			if (robotTargetPosition[i - 1].x == robotTargetPosition[i].x) {
				robotTargetPosition[i].x -= 1;
			}
			else {
				robotTargetPosition[i].x -= 1;
				++readline;
				if (readline < linenum / 2) {
					continue;
				}
				else {
					break;
				}
			}
		}
	}
	else {//line number is odd
		standard = 1;
		readline = 0;
		while (readline < (linenum - 1) / 2) {
			int breakP = 0;
			for (int i = 1; i < robotTargetPosition.size(); ++i) {
				if (robotTargetPosition[i - 1].x == robotTargetPosition[i].x) {
					robotTargetPosition[i - 1].x += 3;
				}
				else {
					robotTargetPosition[i - 1].x += 3;
					++breakP;
					++readline;
					if (breakP >= standard) {
						standard = breakP + 1;
						break;
					}
					else {
						continue;
					}
				}
			}
		}
		standard = 1;
		readline = 0;
		while (readline < (linenum - 1) / 2) {
			int breakP = 0;
			for (int i = robotTargetPosition.size() - 1; i >= 0; --i) {
				if (robotTargetPosition[i - 1].x == robotTargetPosition[i].x) {
					robotTargetPosition[i].x -= 3;
				}
				else {
					robotTargetPosition[i].x -= 3;
					++breakP;
					++readline;
					if (breakP >= standard) {
						standard = breakP + 1;
						break;
					}
					else {
						continue;
					}
				}
			}
		}
	}
}
void InLineSegment() {
	int startPosition = 1;
	int MoveStart = 0;
	for (int i = 0; i < linenum; ++i) {//Operation for every line
		int Singlenum = 1;
		for (int j = startPosition; j < robotTargetPosition.size(); ++j) {
			if (robotTargetPosition[j - 1].x == robotTargetPosition[j].x) {
				++Singlenum;
			}
			else {
				startPosition = j + 1;
				break;
			}
		}//To calculate how many robots in one line
		if (Singlenum % 2 == 0) {//there are even number of robots in one line
			for (int j = MoveStart; j < MoveStart + Singlenum / 2; ++j) {
				robotTargetPosition[j].y -= 2;
				robotTargetPosition[j].LeftFlag = true;
			}
			for (int j = MoveStart + Singlenum / 2; j < MoveStart + Singlenum; ++j) {
				robotTargetPosition[j].y += 1;
			}
		}
		else {
			for (int j = MoveStart; j < MoveStart + (Singlenum - 1) / 2; ++j) {
				robotTargetPosition[j].y -= 3;
				robotTargetPosition[j].LeftFlag = true;
			}
			for (int j = MoveStart + (Singlenum - 1) / 2 + 1; j < MoveStart + Singlenum; ++j) {
				robotTargetPosition[j].y += 3;
			}
		}
		MoveStart = MoveStart + Singlenum;
	}
}
void EvenSeg() {
	int readline = 0;
	int standard = 1;
	while (readline < (linenum / 2) - 1) {
		int breakP = 0;
		for (int i = 1; i < robotTargetPosition.size(); ++i) {
			if (robotTargetPosition[i - 1].x == robotTargetPosition[i].x) {
				robotTargetPosition[i - 1].x += 3;
			}
			else {
				robotTargetPosition[i - 1].x += 3;
				++breakP;
				++readline;
				if (breakP >= standard) {
					standard = breakP + 1;
					break;
				}
				else {
					continue;
				}
			}
		}
	}
	standard = 1;
	readline = 0;
	while (readline < (linenum / 2) - 1) {
		int breakP = 0;
		for (int i = robotTargetPosition.size() - 1; i >= 0; --i) {
			if (robotTargetPosition[i - 1].x == robotTargetPosition[i].x) {
				robotTargetPosition[i].x -= 3;
			}
			else {
				robotTargetPosition[i].x -= 3;
				++breakP;
				++readline;
				if (breakP >= standard) {
					standard = breakP + 1;
					break;
				}
				else {
					continue;
				}
			}
		}
	}
}
void HorSeg() {//Horizontal Segmentation
	int num = 0;
	bool same = false;
	for (int j = 1; j < robotTargetPosition.size(); ++j) {
		if (robotTargetPosition[j - 1].y == robotTargetPosition[j].y - 1 &&
			robotTargetPosition[j - 1].x == robotTargetPosition[j].x) {
			++num;
			same = true;
		}
		else {
			same = false;
			if (num != 0) {
				for (int i = j - num - 1; i <= j - 1; ++i) {
					if (robotTargetPosition[i].LeftFlag == true) {
						robotTargetPosition[i].y -= 3 * (j - i - 1);//correct
					}
					else {
						robotTargetPosition[i].y += 3 * (i - (j - num - 1));
					}
				}
				num = 0;
			}
		}
	}
	if (same == true) {
		for (int i = robotTargetPosition.size() - 1; i > robotTargetPosition.size() - 1 - num; --i) {
			robotTargetPosition[i].y += 3 * (i - (robotTargetPosition.size() - 1 - num));
		}
	}
}
int main() {
	ReadRobotInitPosition();
	ReadMap();
	ReadTask();
	RecordCurrentAndTargetPosition();
	Sleep(1750);
	ChangeMap();

	//Segment and update the target
	InterLineSegment();
	if (linenum % 2 == 0) {
		EvenSeg();
	}
	InLineSegment();
	HorSeg();
	RecordCurrentAndTargetPosition();

	TaskAssign();
	RecordCurrentAndTargetPosition();
	for (int i = 0; i < robotNumber; ++i) {
		if (robotCurrentPosition[i].Dockedflag == true && robotCurrentPosition[i].SetSearched == false) {
			SetSearch(i);
			FindLeader();
		}
	}
	bool Stuckflag = false, FinishFlag = false;
	vector<vector<Point>> planPath;
	for (int i = 0; i < robotNumber; ++i) {
		vector<Point> SinglePath;
		SinglePath = AStarPlan(i, robotCurrentPosition[i], robotTargetPosition[i]);
		planPath.push_back(SinglePath);
	}
	while (FinishFlag == false && Stuckflag == false) {
		int SumofStatus = 0;
		/*cout << "---------------------" << endl;
		PrintMap(4);*/
		for (int i = 0; i < robotNumber; ++i) {
			if (planPath[i].size() > 0) {
				robotPreviousPosition[i] = robotCurrentPosition[i];
				/*if (robotCurrentPosition[i].Dockedflag == false ||
					(robotCurrentPosition[i].Dockedflag == true && robotCurrentPosition[i].Leaderflag == true)) {*/
					robotCurrentPosition[i].x = planPath[i].front().x;
					robotCurrentPosition[i].y = planPath[i].front().y;//重写机器人当前位置，但尚未可视化，可以进行冲突检测避免
				/*}
				else {
					FollowerMove(i);
				}*/
				//Awayfrom();
				CollisionAvoid();
				planPath[i].erase(planPath[i].begin());
				Perception(i);
			}
			else {
				robotCurrentPosition[i].status = 0;
			}
		}
		Sleep(1000);
		RecordCurrentAndTargetPosition();
		ChangeMap();
		for (int i = 0; i < robotNumber; ++i) {
			if (robotCurrentPosition[i].Dockedflag == true && robotCurrentPosition[i].SetSearched == false) {
				SetSearch(i);
				FindLeader();
			}
		}
		vector<vector<Point>>().swap(planPath);
		for (int j = 0; j < robotNumber; ++j) {
			if (robotCurrentPosition[j] == robotTargetPosition[j]) {
				robotCurrentPosition[j].status = 0;
			}
			vector<Point> SinglePath;
			SinglePath = AStarPlan(j, robotCurrentPosition[j], robotTargetPosition[j]);
			planPath.push_back(SinglePath);
			planPath[j].erase(planPath[j].begin());
		}

		for (int j = 0; j < robotNumber; ++j) {
			SumofStatus += robotCurrentPosition[j].status;
		}
		if (num > 30) {
			Stuckflag = true;
			cout << "----------------" << endl;
			std::cout << "Failed" << endl;
		}
		if (SumofStatus == 0) {
			FinishFlag = true;
			cout << "----------------" << endl;
			std::cout << "All Done" << endl;
		}
	}
	return 0;
}