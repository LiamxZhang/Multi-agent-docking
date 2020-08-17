#include "NaiveAlg.h"
#include "RandomNoPair.h"
#include "RandomSearch.h"
#include "WavePropagation.h"
#include "VijayAlg.h"
#include "Log.h"

using namespace std;


int main() {
	// data directory
	string data_home = "../TestRobot/Data/";
	int data_num = 7; 
	vector<string> data_directory;
	for (int i = data_num - 1; i < data_num; ++i) {
		data_directory.push_back(data_home + to_string(i+1) + "/");
	}
	
	// instantiate the algorithm 
	NaiveAlg naive;
	RandomNoPair randno;
	RandomSearch randse;
	WaveAlg waveprop;
	VijayAlg vijay;

	// how many times to run the algorithms
	int epoch_1 = 1;
	int epoch = 20;
	// algorithmns
	//vector<int> alg = {0,0,1,0,0};
	vector<int> alg = { 1,1,1,1,1 };

	// Experiment start
	struct tm t;   //tm结构指针
	time_t now;  //声明time_t类型变量
	time(&now);      //获取系统日期和时间
	localtime_s(&t, &now);   //获取当地日期和时间

	char filename[256] = { 0 };
	sprintf_s(filename, 256, "%d-%d-%d-%d-%d-%d", 1900 + t.tm_year, 1 + t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);

	RecordLog("Experiment start. ", filename);
	for (int i = 0; i < 1; ++i) {
		RecordLog("\nData :\t " + to_string(data_num), filename);
		// NaiveAlg
		if (alg[0]) {
			int success_num = 0;
			int taskstep_total = 0;
			int robotstep_total = 0;
			for (int j = 0; j < epoch_1; ++j) {
				naive.Processing(data_directory[i]);
				//log
				if (naive.isComplete) {
					RecordLog("Naive Algorithm :\t taskstep " + to_string(naive.taskStep)
						+ "\t robotstep " + to_string(naive.robotStep), filename);
					success_num++;
					taskstep_total += naive.taskStep;
					robotstep_total += naive.robotStep;
				}
				else {
					RecordLog("Naive Algorithm fails! ", filename);
				}
			}
			// log mean value
			double success_rate = double(success_num) / double(epoch);
			double taskstep_mean = double(taskstep_total) / double(success_num);
			double robotstep_mean = double(robotstep_total) / double(success_num);

			RecordLog("Data " + to_string(i + 1) + " complete: (Naive Algorithm)\t success rate: "
				+ to_string(success_rate) + "\t task step: " + to_string(taskstep_mean)
				+ "\t robot step: " + to_string(robotstep_mean), filename);
		}
		
		// RandomNoPair
		if (alg[1]) {
			int success_num = 0;
			int taskstep_total = 0;
			int robotstep_total = 0;
			for (int j = 0; j < epoch; ++j) {
				randno.Processing(data_directory[i]);
				//log
				if (randno.isComplete) {
					RecordLog("RandomNoPair Algorithm :\t taskstep " + to_string(randno.taskStep)
						+ "\t robotstep " + to_string(randno.robotStep), filename);
					success_num++;
					taskstep_total += randno.taskStep;
					robotstep_total += randno.robotStep;
				}
				else {
					RecordLog("RandomNoPair Algorithm fails! ", filename);
				}
			}
			// log mean value
			double success_rate = double(success_num) / double(epoch);
			double taskstep_mean = double(taskstep_total) / double(success_num);
			double robotstep_mean = double(robotstep_total) / double(success_num);

			RecordLog("Data " + to_string(i + 1) + " complete: (RandNoPair Algorithm)\t success rate: "
				+ to_string(success_rate) + "\t task step: " + to_string(taskstep_mean)
				+ "\t robot step: " + to_string(robotstep_mean), filename);
		}
		
		// RandomSearch
		if (alg[2]) {
			int success_num = 0;
			int taskstep_total = 0;
			int robotstep_total = 0;
			for (int j = 0; j < epoch; ++j) {
				randse.Processing(data_directory[i]);
				//log
				if (randse.isComplete) {
					RecordLog("RandomSearch Algorithm :\t taskstep " + to_string(randse.taskStep)
						+ "\t robotstep " + to_string(randse.robotStep), filename);
					success_num++;
					taskstep_total += randse.taskStep;
					robotstep_total += randse.robotStep;
				}
				else {
					RecordLog("RandomSearch Algorithm fails! ", filename);
				}
			}
			// log mean value
			double success_rate = double(success_num) / double(epoch);
			double taskstep_mean = double(taskstep_total) / double(success_num);
			double robotstep_mean = double(robotstep_total) / double(success_num);

			RecordLog("Data " + to_string(i + 1) + " complete: (RandomSearch Algorithm)\t success rate: "
				+ to_string(success_rate) + "\t task step: " + to_string(taskstep_mean)
				+ "\t robot step: " + to_string(robotstep_mean), filename);
		}

		// WaveAlg
		if (alg[3]) {
			int success_num = 0;
			int taskstep_total = 0;
			int robotstep_total = 0;
			for (int j = 0; j < epoch; ++j) {
				waveprop.Processing(data_directory[i]);
				//log
				if (waveprop.isComplete) {
					RecordLog("WavePropatation Algorithm :\t taskstep " + to_string(waveprop.taskStep)
						+ "\t robotstep " + to_string(waveprop.robotStep), filename);
					success_num++;
					taskstep_total += waveprop.taskStep;
					robotstep_total += waveprop.robotStep;
				}
				else {
					RecordLog("WavePropatation Algorithm fails! ", filename);
				}
			}
			// log mean value
			double success_rate = double(success_num) / double(epoch);
			double taskstep_mean = double(taskstep_total) / double(success_num);
			double robotstep_mean = double(robotstep_total) / double(success_num);

			RecordLog("Data " + to_string(i + 1) + " complete: (WavePropatation Algorithm)\t success rate: "
				+ to_string(success_rate) + "\t task step: " + to_string(taskstep_mean)
				+ "\t robot step: " + to_string(robotstep_mean), filename);
		}

		// VijayAlg
		if (alg[4]) {
			int success_num = 0;
			int taskstep_total = 0;
			int robotstep_total = 0;
			for (int j = 0; j < epoch_1; ++j) {
				vijay.Processing(data_directory[i]);
				//log
				if (vijay.isComplete) {
					RecordLog("Vijay Algorithm :\t taskstep " + to_string(vijay.taskStep)
						+ "\t robotstep " + to_string(vijay.robotStep), filename);
					success_num++;
					taskstep_total += vijay.taskStep;
					robotstep_total += vijay.robotStep;
				}
				else {
					RecordLog("Vijay Algorithm fails! ", filename);
				}
			}
			// log mean value
			double success_rate = double(success_num) / double(epoch);
			double taskstep_mean = double(taskstep_total) / double(success_num);
			double robotstep_mean = double(robotstep_total) / double(success_num);

			RecordLog("Data " + to_string(i + 1) + " complete: (Vijay Algorithm)\t success rate: "
				+ to_string(success_rate) + "\t task step: " + to_string(taskstep_mean)
				+ "\t robot step: " + to_string(robotstep_mean), filename);
		}
		
	}
}
