//
//  basic.hpp
//  sceneReconstruct
//
//  Created by 周晓宇 on 4/5/17.
//  Copyright © 2017 xiaoyu. All rights reserved.
//

#ifndef basic_hpp
#define basic_hpp

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include<opencv2/core/core.hpp>

using namespace std;

void LoadImages(const string &strPathLeft,
                const string &strPathRight,
                const string &strPathSeq,
                vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight);




#endif /* basic_hpp */
