#include "NaiveAlg.h"
#include "RandomNoPair.h"
#include "RandomSearch.h"
#include "WavePropagation.h"
#include "VijayAlg.h"
#include "Log.h"

using namespace std;


int main() {
	// instantiate the algorithm 
	NaiveAlg naive;
	RandomNoPair randno;
	RandomSearch randse;
	WaveAlg waveprop;
	VijayAlg vijay;

	// how many times to run the algorithms
	int epoch_1 = 1;
	int epoch = 20;
	vector<int> alg = { 0,1,1,0,0 };

	// Experiment start
	struct tm t;   //tm结构指针
	time_t now;  //声明time_t类型变量
	time(&now);      //获取系统日期和时间
	localtime_s(&t, &now);   //获取当地日期和时间
	char filename[256] = { 0 };
	sprintf_s(filename, 256, "%d-%d-%d-%d-%d-%d", 1900 + t.tm_year, 1 + t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
	RecordLog("Experiment start. ", filename);

	// data directory
	string data_home = "../TestRobot/Data/";
	int data_start = 6;
	int data_end = 11;
	for (int data_num = data_start; data_num < data_end; ++data_num) {
		string data_directory = data_home + to_string(data_num) + "/";
		RecordLog("\nData :\t " + to_string(data_num), filename);

		// NaiveAlg
		if (alg[0]) {
			double success_num = 0;

			double taskstep_total = 0;
			double robotstep_total = 0;

			double taskstep_mean_total = 0;
			double robotstep_mean_total = 0;

			double taskstep_var_total = 0;
			double robotstep_var_total = 0;

			double taskstep_sys_total = 0;
			double robotstep_sys_total = 0;

			for (int j = 0; j < epoch_1; ++j) {
				naive.Processing(data_directory);
				//log
				if (naive.isComplete) {
					RecordLog("Naive Algorithm :\t taskstep " + to_string(naive.taskStep)
						+ "\t taskstep mean " + to_string(naive.taskStep_mean)
						+ "\t taskstep variance " + to_string(naive.taskStep_variance)
						+ "\t total taskstep " + to_string(naive.taskStep_sys)
						+ "\t robotstep " + to_string(naive.robotStep)
						+ "\t robotstep mean " + to_string(naive.robotStep_mean)
						+ "\t robotstep variance " + to_string(naive.robotStep_variance)
						+ "\t total robotstep " + to_string(naive.robotStep_sys), filename);
					success_num++;
					taskstep_total += naive.taskStep;
					robotstep_total += naive.robotStep;
					taskstep_mean_total += naive.taskStep_mean;
					robotstep_mean_total += naive.robotStep_mean;
					taskstep_var_total += naive.taskStep_variance;
					robotstep_var_total += naive.robotStep_variance;
					taskstep_sys_total += naive.taskStep_sys;
					robotstep_sys_total += naive.robotStep_sys;
				}
				else {
					RecordLog("Naive Algorithm fails! ", filename);
				}
			}
			// log mean value
			double success_rate = double(success_num) / double(epoch_1);
			double taskstep_ = double(taskstep_total) / double(success_num);
			double robotstep_ = double(robotstep_total) / double(success_num);
			double taskstep_mean_ = double(taskstep_mean_total) / double(success_num);
			double robotstep_mean_ = double(robotstep_mean_total) / double(success_num);
			double taskstep_var_ = double(taskstep_var_total) / double(success_num);
			double robotstep_var_ = double(robotstep_var_total) / double(success_num);
			double taskstep_sys_ = double(taskstep_sys_total) / double(success_num);
			double robotstep_sys_ = double(robotstep_sys_total) / double(success_num);

			RecordLog("Data " + to_string(data_num) + " complete: (Naive Algorithm)\t success rate: "
				+ to_string(success_rate) + "\t task step: " + to_string(taskstep_)
				+ "\t taskstep mean " + to_string(taskstep_mean_)
				+ "\t taskstep variance " + to_string(taskstep_var_)
				+ "\t total taskstep " + to_string(taskstep_sys_)
				+ "\t robotstep " + to_string(robotstep_)
				+ "\t robotstep mean " + to_string(robotstep_mean_)
				+ "\t robotstep variance " + to_string(robotstep_var_)
				+ "\t total robotstep " + to_string(robotstep_sys_), filename);
		}

		// RandomNoPair
		if (alg[1]) {
			double success_num = 0;

			double taskstep_total = 0;
			double robotstep_total = 0;

			double taskstep_mean_total = 0;
			double robotstep_mean_total = 0;

			double taskstep_var_total = 0;
			double robotstep_var_total = 0;

			double taskstep_sys_total = 0;
			double robotstep_sys_total = 0;
			for (int j = 0; j < epoch; ++j) {
				randno.Processing(data_directory);
				//log
				if (randno.isComplete) {
					RecordLog("RandomNoPair Algorithm :\t taskstep " + to_string(randno.taskStep)
						+ "\t taskstep mean " + to_string(randno.taskStep_mean)
						+ "\t taskstep variance " + to_string(randno.taskStep_variance)
						+ "\t total taskstep " + to_string(randno.taskStep_sys)
						+ "\t robotstep " + to_string(randno.robotStep)
						+ "\t robotstep mean " + to_string(randno.robotStep_mean)
						+ "\t robotstep variance " + to_string(randno.robotStep_variance)
						+ "\t total robotstep " + to_string(randno.robotStep_sys), filename);
					success_num++;
					taskstep_total += randno.taskStep;
					robotstep_total += randno.robotStep;
					taskstep_mean_total += randno.taskStep_mean;
					robotstep_mean_total += randno.robotStep_mean;
					taskstep_var_total += randno.taskStep_variance;
					robotstep_var_total += randno.robotStep_variance;
					taskstep_sys_total += randno.taskStep_sys;
					robotstep_sys_total += randno.robotStep_sys;
				}
				else {
					RecordLog("RandomNoPair Algorithm fails! ", filename);
				}
			}
			// log mean value
			double success_rate = double(success_num) / double(epoch);
			double taskstep_ = double(taskstep_total) / double(success_num);
			double robotstep_ = double(robotstep_total) / double(success_num);
			double taskstep_mean_ = double(taskstep_mean_total) / double(success_num);
			double robotstep_mean_ = double(robotstep_mean_total) / double(success_num);
			double taskstep_var_ = double(taskstep_var_total) / double(success_num);
			double robotstep_var_ = double(robotstep_var_total) / double(success_num);
			double taskstep_sys_ = double(taskstep_sys_total) / double(success_num);
			double robotstep_sys_ = double(robotstep_sys_total) / double(success_num);

			RecordLog("Data " + to_string(data_num) + " complete: (RandNoPair Algorithm)\t success rate: "
				+ to_string(success_rate) + "\t task step: " + to_string(taskstep_)
				+ "\t taskstep mean " + to_string(taskstep_mean_)
				+ "\t taskstep variance " + to_string(taskstep_var_)
				+ "\t total taskstep " + to_string(taskstep_sys_)
				+ "\t robotstep " + to_string(robotstep_)
				+ "\t robotstep mean " + to_string(robotstep_mean_)
				+ "\t robotstep variance " + to_string(robotstep_var_)
				+ "\t total robotstep " + to_string(robotstep_sys_), filename);
		}

		// RandomSearch
		if (alg[2]) {
			double success_num = 0;

			double taskstep_total = 0;
			double robotstep_total = 0;

			double taskstep_mean_total = 0;
			double robotstep_mean_total = 0;

			double taskstep_var_total = 0;
			double robotstep_var_total = 0;

			double taskstep_sys_total = 0;
			double robotstep_sys_total = 0;
			for (int j = 0; j < epoch; ++j) {
				randse.Processing(data_directory);
				//log
				if (randse.isComplete) {
					RecordLog("RandomSearch Algorithm :\t taskstep " + to_string(randse.taskStep)
						+ "\t taskstep mean " + to_string(randse.taskStep_mean)
						+ "\t taskstep variance " + to_string(randse.taskStep_variance)
						+ "\t total taskstep " + to_string(randse.taskStep_sys)
						+ "\t robotstep " + to_string(randse.robotStep)
						+ "\t robotstep mean " + to_string(randse.robotStep_mean)
						+ "\t robotstep variance " + to_string(randse.robotStep_variance)
						+ "\t total robotstep " + to_string(randse.robotStep_sys), filename);
					success_num++;
					taskstep_total += randse.taskStep;
					robotstep_total += randse.robotStep;
					taskstep_mean_total += randse.taskStep_mean;
					robotstep_mean_total += randse.robotStep_mean;
					taskstep_var_total += randse.taskStep_variance;
					robotstep_var_total += randse.robotStep_variance;
					taskstep_sys_total += randse.taskStep_sys;
					robotstep_sys_total += randse.robotStep_sys;
				}
				else {
					RecordLog("RandomSearch Algorithm fails! ", filename);
				}
			}
			// log mean value
			double success_rate = double(success_num) / double(epoch);
			double taskstep_ = double(taskstep_total) / double(success_num);
			double robotstep_ = double(robotstep_total) / double(success_num);
			double taskstep_mean_ = double(taskstep_mean_total) / double(success_num);
			double robotstep_mean_ = double(robotstep_mean_total) / double(success_num);
			double taskstep_var_ = double(taskstep_var_total) / double(success_num);
			double robotstep_var_ = double(robotstep_var_total) / double(success_num);
			double taskstep_sys_ = double(taskstep_sys_total) / double(success_num);
			double robotstep_sys_ = double(robotstep_sys_total) / double(success_num);

			RecordLog("Data " + to_string(data_num) + " complete: (RandomSearch Algorithm)\t success rate: "
				+ to_string(success_rate) + "\t task step: " + to_string(taskstep_)
				+ "\t taskstep mean " + to_string(taskstep_mean_)
				+ "\t taskstep variance " + to_string(taskstep_var_)
				+ "\t total taskstep " + to_string(taskstep_sys_)
				+ "\t robotstep " + to_string(robotstep_)
				+ "\t robotstep mean " + to_string(robotstep_mean_)
				+ "\t robotstep variance " + to_string(robotstep_var_)
				+ "\t total robotstep " + to_string(robotstep_sys_), filename);
		}

		// WaveAlg
		if (alg[3]) {
			double success_num = 0;

			double taskstep_total = 0;
			double robotstep_total = 0;

			double taskstep_mean_total = 0;
			double robotstep_mean_total = 0;

			double taskstep_var_total = 0;
			double robotstep_var_total = 0;

			double taskstep_sys_total = 0;
			double robotstep_sys_total = 0;
			for (int j = 0; j < epoch; ++j) {
				waveprop.Processing(data_directory);
				//log
				if (waveprop.isComplete) {
					RecordLog("WavePropatation Algorithm :\t taskstep " + to_string(waveprop.taskStep)
						+ "\t taskstep mean " + to_string(waveprop.taskStep_mean)
						+ "\t taskstep variance " + to_string(waveprop.taskStep_variance)
						+ "\t total taskstep " + to_string(waveprop.taskStep_sys)
						+ "\t robotstep " + to_string(waveprop.robotStep)
						+ "\t robotstep mean " + to_string(waveprop.robotStep_mean)
						+ "\t robotstep variance " + to_string(waveprop.robotStep_variance)
						+ "\t total robotstep " + to_string(waveprop.robotStep_sys), filename);
					success_num++;
					taskstep_total += waveprop.taskStep;
					robotstep_total += waveprop.robotStep;
					taskstep_mean_total += waveprop.taskStep_mean;
					robotstep_mean_total += waveprop.robotStep_mean;
					taskstep_var_total += waveprop.taskStep_variance;
					robotstep_var_total += waveprop.robotStep_variance;
					taskstep_sys_total += waveprop.taskStep_sys;
					robotstep_sys_total += waveprop.robotStep_sys;
				}
				else {
					RecordLog("WavePropatation Algorithm fails! ", filename);
				}
			}
			// log mean value
			double success_rate = double(success_num) / double(epoch);
			double taskstep_ = double(taskstep_total) / double(success_num);
			double robotstep_ = double(robotstep_total) / double(success_num);
			double taskstep_mean_ = double(taskstep_mean_total) / double(success_num);
			double robotstep_mean_ = double(robotstep_mean_total) / double(success_num);
			double taskstep_var_ = double(taskstep_var_total) / double(success_num);
			double robotstep_var_ = double(robotstep_var_total) / double(success_num);
			double taskstep_sys_ = double(taskstep_sys_total) / double(success_num);
			double robotstep_sys_ = double(robotstep_sys_total) / double(success_num);

			RecordLog("Data " + to_string(data_num) + " complete: (WavePropatation Algorithm)\t success rate: "
				+ to_string(success_rate) + "\t task step: " + to_string(taskstep_)
				+ "\t taskstep mean " + to_string(taskstep_mean_)
				+ "\t taskstep variance " + to_string(taskstep_var_)
				+ "\t total taskstep " + to_string(taskstep_sys_)
				+ "\t robotstep " + to_string(robotstep_)
				+ "\t robotstep mean " + to_string(robotstep_mean_)
				+ "\t robotstep variance " + to_string(robotstep_var_)
				+ "\t total robotstep " + to_string(robotstep_sys_), filename);
		}

		// VijayAlg
		if (alg[4]) {
			double success_num = 0;

			double taskstep_total = 0;
			double robotstep_total = 0;

			double taskstep_mean_total = 0;
			double robotstep_mean_total = 0;

			double taskstep_var_total = 0;
			double robotstep_var_total = 0;

			double taskstep_sys_total = 0;
			double robotstep_sys_total = 0;
			for (int j = 0; j < epoch_1; ++j) {
				vijay.Processing(data_directory);
				//log
				if (vijay.isComplete) {
					RecordLog("Vijay Algorithm :\t taskstep " + to_string(vijay.taskStep)
						+ "\t taskstep mean " + to_string(vijay.taskStep_mean)
						+ "\t taskstep variance " + to_string(vijay.taskStep_variance)
						+ "\t total taskstep " + to_string(vijay.taskStep_sys)
						+ "\t robotstep " + to_string(vijay.robotStep)
						+ "\t robotstep mean " + to_string(vijay.robotStep_mean)
						+ "\t robotstep variance " + to_string(vijay.robotStep_variance)
						+ "\t total robotstep " + to_string(vijay.robotStep_sys), filename);
					success_num++;
					taskstep_total += vijay.taskStep;
					robotstep_total += vijay.robotStep;
					taskstep_mean_total += vijay.taskStep_mean;
					robotstep_mean_total += vijay.robotStep_mean;
					taskstep_var_total += vijay.taskStep_variance;
					robotstep_var_total += vijay.robotStep_variance;
					taskstep_sys_total += vijay.taskStep_sys;
					robotstep_sys_total += vijay.robotStep_sys;
				}
				else {
					RecordLog("Vijay Algorithm fails! ", filename);
				}
			}
			// log mean value
			double success_rate = double(success_num) / double(epoch_1);
			double taskstep_ = double(taskstep_total) / double(success_num);
			double robotstep_ = double(robotstep_total) / double(success_num);
			double taskstep_mean_ = double(taskstep_mean_total) / double(success_num);
			double robotstep_mean_ = double(robotstep_mean_total) / double(success_num);
			double taskstep_var_ = double(taskstep_var_total) / double(success_num);
			double robotstep_var_ = double(robotstep_var_total) / double(success_num);
			double taskstep_sys_ = double(taskstep_sys_total) / double(success_num);
			double robotstep_sys_ = double(robotstep_sys_total) / double(success_num);

			RecordLog("Data " + to_string(data_num) + " complete: (Vijay Algorithm)\t success rate: "
				+ to_string(success_rate) + "\t task step: " + to_string(taskstep_)
				+ "\t taskstep mean " + to_string(taskstep_mean_)
				+ "\t taskstep variance " + to_string(taskstep_var_)
				+ "\t total taskstep " + to_string(taskstep_sys_)
				+ "\t robotstep " + to_string(robotstep_)
				+ "\t robotstep mean " + to_string(robotstep_mean_)
				+ "\t robotstep variance " + to_string(robotstep_var_)
				+ "\t total robotstep " + to_string(robotstep_sys_), filename);
		}

	}
}
