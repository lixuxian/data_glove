#include "data_dumper.h"
#include <fstream>
#include <iostream>
using namespace std;

void dump(char *filename, char *data) {
	ofstream fout(filename, ios::app);
	if (!fout) {
		cout << "open file error!"<< endl;
		return;
	}
	fout << data << endl;
	fout.close();
}
