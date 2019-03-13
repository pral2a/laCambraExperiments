#pragma once

#include "ofMain.h"
#include "ofxPointCloud.h"

class ofPointsStorer : public ofThread {
    
    public:

    ofThreadChannel<ofPointCloud> storer;

    void threadedFunction();


};