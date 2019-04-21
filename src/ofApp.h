#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxVideoRecorder.h"
#include "ofxPointCloud.h"
#include "ofxBinaryMesh.h"

// Windows users:
// You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:
//
//     ofxKinect/libs/libfreenect/platform/windows/inf
//
// This should install the Kinect camera, motor, & audio drivers.
//
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager.
//
// No way around the Windows driver dance, sorry.

class ofApp : public ofBaseApp, public ofThread{
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawInstructions();
	void drawPointCloud();
	void drawFakePointCloud();

	void drawFilm();
	void drawPreview();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	ofPrimitiveMode meshMode();
	
	ofxKinect kinect;
		
	bool bThreshWithOpenCV;
	bool bControlsOverlay;
	bool bPrevRealSize;
	bool bDrawVertices;
	bool bDrawWireframe;
	bool bDrawFaces;
	bool bViewOrbit;
	bool bProxyMode;

	int nearThreshold;
	int farThreshold;
	
	float panAngle;
	float tiltAngle;
	int stepRes;
	int angle;
	int sMeshMode;
	float pointSize;

	int frameNumber;
	int frameNumberSent;
	int frameLoaded;

	long lastSavedFrame;

	ofPointCloud pointCloud;
	ofPointCloud pointCloudPreLoaded;


	ofEasyCam easyCam;
    
    ofxVideoRecorder vidRecorder;

	ofFbo film;

	ofPixels filmFrame;

    bool bRecording;
    bool bEncoding;
    bool bWrittingPoints;
    bool bReplay;
    void startRecord();
    void stopRecord();
    void recordFilm();
    void recordingComplete(ofxVideoRecorderOutputFileCompleteEventArgs& args);
   
   	int const totalWorkers = 20;
	ofSaveWorker W[20];  

    void createTakeDirectory();

    void loadTake();

    string fixPath;

    string takeDirPath;
    string pointsDirPath;
    string takeName;


    long previousFrameTime;
    long previousSavedFrameTime;
    long previousFolderCheckTime;
	long previousEnlapsedFrameTime;

	bool bNewFrame;

    bool bReplayPause;

    int frameTime;

    bool t;

    int frameCompensation;

    int frameThing;

};