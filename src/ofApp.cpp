#include "ofApp.h"

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

//	film.allocate(768, 768, GL_RGBA);

    vidRecorder.setVideoCodec("prores");
//    vidRecorder.setVideoBitrate("800k");
    ofAddListener(vidRecorder.outputFileCompleteEvent, this, &ofApp::recordingComplete);

	film.allocate(2048, 1080, GL_RGB);

	film.begin();
    ofClear(255,255,255, 0);
    film.end();

	// enable depth->video image calibration
	// kinect.setRegistration(true);

	// kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	// kinect.open();		// opens first available kinect

	// print the intrinsic IR sensor values
	// if(kinect.isConnected()) {
	// 	ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
	// 	ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
	// 	ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
	// 	ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	// }

	bControlsOverlay = false;
    bRecording = false;

	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	// kinect.setCameraTiltAngle(angle);

    vidRecorder.setFfmpegLocation(ofFilePath::getAbsolutePath("/usr/local/bin/ffmpeg"));
}

//--------------------------------------------------------------
void ofApp::update() {
	// kinect.update();

	// there is a new frame and we are connected
	// if(kinect.isFrameNew()) {

	// }

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(0, 0, 0);

	drawFilm();
	recordFilm();

	film.draw(0,0);

	if(bControlsOverlay) {
		drawInstructions();
	}

    if(bRecording){
    	ofSetColor(255, 0, 0);
    	ofDrawCircle(ofGetWidth() - 20, 20, 5);
    }

}

void ofApp::recordFilm(){
	// Storing an image works fine
	// ofSaveImage(filmFrame, "save.png", OF_IMAGE_QUALITY_BEST);

	if(bRecording){

		film.readToPixels(filmFrame);

		bool success = vidRecorder.addFrame(filmFrame);

	    if (!success) {
	        ofLogWarning("This frame was not added!");
	    }

	    if (vidRecorder.hasVideoError()) {
	        ofLogWarning("The video recorder failed to write some frames!");
	    }
	}

}

void ofApp::recordingComplete(ofxVideoRecorderOutputFileCompleteEventArgs& args){
    ofLogWarning("The recoded video file is now complete.");
}

void ofApp::drawFakePointCloud() {
	int width = 640;
	int height = 480;
	ofMesh mesh;
	int step = 5;
	for (int y = 0; y < height; y += step){
	    for (int x = 0; x<width; x += step){
	        mesh.addVertex(ofPoint(x,y,0)); // make a new vertex
	        mesh.addColor(ofFloatColor(255,255,255));  // add a color at that vertex
	    }
	}

	glPointSize(3);
	ofPushMatrix();
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

/*void ofApp::drawPointCloud() {
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
}*/

void ofApp::drawFilm(){
	film.begin();
	ofBackground(0, 0, 0);
	easyCam.begin();
	drawFakePointCloud();
	easyCam.end();
	film.end();
}

//--------------------------------------------------------------
void ofApp::exit() {
    ofRemoveListener(vidRecorder.outputFileCompleteEvent, this, &ofApp::recordingComplete);
	// kinect.setCameraTiltAngle(0); // zero the tilt on exit
	// kinect.close();
    stopRecord();
}

//--------------------------------------------------------------

void ofApp::startRecord() {
    if(!vidRecorder.isInitialized()) {
    	bRecording = true;
    	ofLogNotice() << "Recording!";
        vidRecorder.setup(ofGetTimestampString()+"laCambra.mov", film.getWidth(), film.getHeight(), 30);
    }

    vidRecorder.start();

}

void ofApp::stopRecord() {
    if(bRecording) {
    	bRecording = false;
    	ofLogNotice() << "Stop Recording!";
    	vidRecorder.close();
 	}
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case 'r':
			startRecord();
			break;
		case 's':
			stopRecord();
			break;
		case 'q':
			bControlsOverlay = !bControlsOverlay;
			break;
	}
}

//--------------------------------------------------------------
void ofApp::drawInstructions() {
		// draw instructions
	ofSetColor(255, 255, 255);

	stringstream reportStream;

	reportStream << "fps: " << ofGetFrameRate() << endl;
	// << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if (bRecording) {
    	reportStream << "video queue size: " << vidRecorder.getVideoQueueSize() << endl;
    }

    reportStream << (bRecording?"pause":"start") << " recording: r" << endl;
    reportStream << (bRecording?"close current video file: s":"") << endl;

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

