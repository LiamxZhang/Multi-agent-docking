#pragma once
// 日志记录存储在Log.txt中
#include <iostream>
#include <fstream> 
//#include <ctime>

using namespace std;

bool RecordLog(string s, string fname = "Log") {
	ofstream logFile;
	//time_t now = time(0);
	//char* ch = ctime_s(&now);
	//string dt = ch;

	logFile.open("../TestRobot/Log" + fname + ".txt", ofstream::app);
	if (logFile) {
		logFile << s.c_str() << endl;
		logFile.close();
	}
	else {
		cout << " Failed to open the target file . The target file is Log.txt. " << endl;
		return false;
	}
	return true;
}