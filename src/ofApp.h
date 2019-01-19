#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxVideoRecorder.h"

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

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawInstructions();
	void drawPointCloud();
	void drawFilm();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
		
	bool bThreshWithOpenCV;
	bool bControlsOverlay;

	int nearThreshold;
	int farThreshold;
	
	int angle;

	long lastSavedFrame;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
    
    ofxVideoRecorder vidRecorder;

	ofFbo film;

	ofPixels filmFrame;

    bool bRecording;
    void startRecord();
    void stopRecord();
};
