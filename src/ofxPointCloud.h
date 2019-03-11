#include "ofMesh.h"
#include "ofLog.h"
#include "ofxBinaryMesh.h"

class ofPointCloud : public ofMesh {
    
    public:

    string path;

    void setSavePath(string _path){
        path = _path;
    }    

    string getSavePath(){
        return path;
    }

    // void save(const std::filesystem::path& path, bool useBinary = false){
    //     ofLogWarning("ofPointCloud") << "hello: " << meshNumber;
    // }

    // private:


};

class ofSaveWorker : public ofThread {
    
    public:

    ofThreadChannel<ofPointCloud> pointsSaver;

    void threadedFunction() {
        ofPointCloud pointCloud;
        while(pointsSaver.receive(pointCloud)){
            ofxBinaryMesh::save(pointCloud.getSavePath(), pointCloud);
            // pointCloud.save(pointCloud.getSavePath());
        }
    }


};
