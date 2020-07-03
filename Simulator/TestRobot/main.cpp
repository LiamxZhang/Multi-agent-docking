#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include <ctime>
#include <math.h>
#include <algorithm>
#include <windows.h>

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

	// run the algorithms
	//naive.Processing();
	//randno.Processing(); //
	randse.Processing();
	//waveprop.Processing(); // 有些距离不到3
	//vijay.Processing();

	// statistics in the class


}
