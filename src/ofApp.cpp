#include "ofApp.h"

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

//	film.allocate(768, 768, GL_RGBA);

	film.allocate(768, 768);

	film.begin();
    ofClear(255,255,255, 0);
    film.end();

	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	kinect.open();		// opens first available kinect

	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}

	bControlsOverlay = false;

	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

    vidRecorder.setFfmpegLocation(ofFilePath::getAbsolutePath("/usr/local/bin/ffmpeg"));

    startRecord();

    lastSavedFrame = ofGetElapsedTimeMillis();

}

//--------------------------------------------------------------
void ofApp::update() {
	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

	}

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(0, 0, 0);

	drawFilm();

	// TODO: Testing video output. We'll test using static images instead.

	if (ofGetElapsedTimeMillis() - lastSavedFrame > 1000) {


		film.readToPixels(filmFrame);

		bool success = vidRecorder.addFrame(filmFrame);

	    if (!success) {
	        ofLogWarning("This frame was not added!");
	    } else {
	    	ofLogWarning("Saved!");
	    	lastSavedFrame = ofGetElapsedTimeMillis();
	    }

	    if (vidRecorder.hasVideoError()) {
	        ofLogWarning("The video recorder failed to write some frames!");
	    }


	}

	film.draw(0,0);

	if(bControlsOverlay) {
		drawInstructions();
	}

}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(ofColor(255,255,255));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

void ofApp::drawFilm(){
	film.begin();
	ofBackground(0, 0, 0);
	easyCam.begin();
	drawPointCloud();
	easyCam.end();
	film.end();
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
    stopRecord();
}

//--------------------------------------------------------------

void ofApp::startRecord() {

    bRecording = true;

    if(bRecording && !vidRecorder.isInitialized()) {
        vidRecorder.setup("laCambra.mp4", 768, 768, 30, 44100, 2);
    }

    vidRecorder.start();

}

void ofApp::stopRecord() {
    bRecording = false;
    vidRecorder.close();

}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case 'q':
			bControlsOverlay = !bControlsOverlay;
			break;
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;

		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;

		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;

		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;

		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;

		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;

		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;

		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void ofApp::drawInstructions() {
		// draw instructions
	ofSetColor(255, 255, 255);

	stringstream reportStream;

    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }

	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }

	ofDrawBitmapString(reportStream.str(), 20, 652);
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}

