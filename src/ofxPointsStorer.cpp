#include "ofxPointsStorer.h"

void ofPointsStorer::threadedFunction() {
    ofPointCloud pointCloud;
    while(storer.receive(pointCloud)){
        pointCloud.saveRaw();
    }
}