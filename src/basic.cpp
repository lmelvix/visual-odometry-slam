#include "basic.hpp"

void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathSeq, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight) {
    ifstream fTimes;
    fTimes.open(strPathSeq.c_str());
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof()) {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
        }
    }
}
