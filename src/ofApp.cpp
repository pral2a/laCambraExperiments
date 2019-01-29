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

	//film.allocate(3840, 2160, GL_RGB);

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
    bRecording = false;
    bPrevRealSize = false;

	bDrawVertices = true;
	bDrawWireframe = false;
	bDrawFaces = false;
	bViewOrbit = false;

	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	sMeshMode = 0;
	panAngle = 0;
	tiltAngle = 0;
	pointSize = 3.0;

	stepRes = 2;

	frameNumber = 0;

    vidRecorder.setFfmpegLocation(ofFilePath::getAbsolutePath("/usr/local/bin/ffmpeg"));

	waitForThread(true);
}

//--------------------------------------------------------------
void ofApp::update() {

	kinect.update();

	if(kinect.isFrameNew()) {
		recordFilm();
	}

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(0, 0, 0);

	drawFilm();

	drawPreview();

	if(bControlsOverlay) {
		drawInstructions();
	}

    if(bRecording){
    	ofSetColor(255, 0, 0);
    	ofDrawCircle(ofGetWidth() - 20, 20, 5);
    }

}

void ofApp::drawPreview(){
	if (bPrevRealSize) {

		int xPos = (ofGetWidth() - film.getWidth())/2;
		int yPos = (ofGetHeight() - film.getHeight())/2;

		film.draw(xPos, yPos);
	} else {
		film.draw(0,0, ofGetWidth(), ofGetHeight());
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

/*void ofApp::drawFakePointCloud() {
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
}*/

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;

	mesh.setMode(meshMode());

	if (stepRes < 2) stepRes = 2;
	int step = stepRes;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(ofColor(255,255,255));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}

    if(bRecording) pointsSaver.send(mesh);

	glPointSize(pointSize);
	ofPushMatrix();
	ofRotateY(panAngle);
	ofRotateX(tiltAngle);
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit


	ofEnableDepthTest();

	if (bDrawVertices) mesh.drawVertices();
	if (bDrawWireframe) mesh.drawWireframe();
	if (bDrawFaces) mesh.drawFaces();

	ofDisableDepthTest();
	ofPopMatrix();
}

ofPrimitiveMode ofApp::meshMode(){
	ofPrimitiveMode meshModes[7] = {OF_PRIMITIVE_POINTS, OF_PRIMITIVE_TRIANGLES, OF_PRIMITIVE_TRIANGLE_STRIP, OF_PRIMITIVE_TRIANGLE_FAN, OF_PRIMITIVE_LINES, OF_PRIMITIVE_LINE_STRIP, OF_PRIMITIVE_LINE_LOOP};
	return meshModes[sMeshMode];
}

void ofApp::drawFilm(){


	film.begin();
	ofBackground(0, 0, 0);
	easyCam.begin();
	drawPointCloud();
	easyCam.end();
	film.end();
}

void ofApp::threadedFunction(){
    ofMesh pointCloud;
    while(pointsSaver.receive(pointCloud)){
    	frameNumber++;
    	char fileName[13];
    	sprintf(fileName,"%06d-pc.ply",frameNumber);
        pointCloud.save(ofToDataPath(fileName), true);
    }
}

//--------------------------------------------------------------
void ofApp::exit() {
    ofRemoveListener(vidRecorder.outputFileCompleteEvent, this, &ofApp::recordingComplete);
	// kinect.setCameraTiltAngle(0); // zero the tilt on exit
	// kinect.close();
    stopRecord();
    pointsSaver.close();
  	waitForThread(true);
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
		case 'v':
			bPrevRealSize = !bPrevRealSize;
			break;
		case '1':
			bDrawVertices = !bDrawVertices;
			break;
		case '2':
			bDrawFaces = !bDrawFaces;
			break;
		case '3':
			bDrawWireframe = !bDrawWireframe;
			break;
		case 'm':
			if(sMeshMode > 7) sMeshMode = 0;
			sMeshMode++;
			break;
		case 'p':
			pointSize += 0.1f;
			break;
		case 'l':
			pointSize -= 0.1f;
			break;
		case 'o':
			stepRes++;
			break;
		case 'k':
			stepRes--;
			break;
		case OF_KEY_RIGHT:
     		panAngle += 0.100f;
			break;
		case OF_KEY_LEFT:
     		panAngle -= 0.100f;
			break;
		case OF_KEY_UP:
     		tiltAngle += 1.500f;
			break;
		case OF_KEY_DOWN:
     		tiltAngle -= 1.500f;
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

    reportStream << "point size: " << pointSize << endl;

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

