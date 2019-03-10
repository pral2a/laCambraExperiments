#include "ofMesh.h"
#include "ofLog.h"

class ofPointCloud : public ofMesh {
    
    public:

    int meshNumber;

    void setMeshNumber(int _meshNumber){
        meshNumber = _meshNumber;
    }    

    int getMeshNumber(){
        return meshNumber;
    }

    // void save(const std::filesystem::path& path, bool useBinary = false){
    //     ofLogWarning("ofPointCloud") << "hello: " << meshNumber;
    // }

    // private:


};
