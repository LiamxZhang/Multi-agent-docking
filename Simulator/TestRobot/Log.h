#pragma once
// 日志记录存储在Log.txt中
#include <iostream>
#include <fstream> 

using namespace std;

bool RecordLog(string s) {
	ofstream logFile;
	logFile.open("../TestRobot/Log.txt", ofstream::app);
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