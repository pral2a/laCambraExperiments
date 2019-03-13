#pragma once

#include "ofMain.h"
#include "ofxPointCloud.h"
#include "ofxPointsStorer.h"

class ofPointsStorerPool {

    private:

    ofPointsStorer pointStorer[5];
    int totalWorkers = 5;

    public:

    void start();
    void saveRaw(ofPointCloud& pointCloud);
    void stop();

    // ofPointsStorerPool();
};