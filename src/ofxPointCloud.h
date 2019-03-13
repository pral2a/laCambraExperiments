#pragma once

#include "ofMain.h"

class ofPointCloud : public ofMesh {

    private:

    string path;
    int frameNumber;
    
    public:

    void setFrameNumber(int _frameNumber);
    int getFrameNumber();

    void setSavePath(string _path);
    string getSavePath();

    void saveRaw();
    bool loadRaw(const string& _path);

};