#include "ofxPointsStorerPool.h"

void ofPointsStorerPool::start() {
    for(int i = 0; i < totalWorkers; i++){
        pointStorer[i].startThread();
    }
}

void ofPointsStorerPool::saveRaw(ofPointCloud& pointCloud) {
    int const worker = pointCloud.getFrameNumber() % totalWorkers;
    pointStorer[worker].storer.send(pointCloud);
}

void ofPointsStorerPool::stop(){
    for(int i = 0; i < totalWorkers ; i++){
        pointStorer[i].storer.close();
        pointStorer[i].waitForThread(true);
    }
}

