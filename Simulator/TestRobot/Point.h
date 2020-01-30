#pragma once
// 定义底层地图点的数据格式，基于Point类可定义Task，Robot等类
// 每个点主要特征是x,y坐标，地图的origin位于其左下角

class Point {         // the point in the map
public:
	Point() :x(0), y(0) {}
	Point(int tx, int ty) {                      // target points
		x = tx;
		y = ty;
	}
	bool operator==(const Point& r) const {     // reload logic operation ==
		return (x == r.x) && (y == r.y);
	}
	bool operator < (const Point& r) const {    // reload logic operation <
		int a = x * x + y * y;
		int b = r.x * r.x + r.y * r.y;
		if (a < b) {
			return true;
		}
		else if (a == b) {                      // if a and b have the same distance to origin, less with less x
			if (x < r.x)                        // pay attention
				return true;
			else
				return false;
		}
		else {
			return false;
		}
	}
	Point up() {
		return Point(x, y + 1);
	}
	Point down() {
		return Point(x, y - 1);
	}
	Point left() {
		return Point(x - 1, y);
	}
	Point right() {
		return Point(x + 1, y);
	}
	// friend with all other classes
	friend class Rule;
	friend class Robot;
	friend class Task;
	friend int main();
	int x;
	int y;
	//bool Dockedflag;
	//bool Leaderflag;
	//bool Taskflag;  
	//bool Robotflag;
	//bool Obstacleflag;
	//int LeaderNum = -1;
private:
};