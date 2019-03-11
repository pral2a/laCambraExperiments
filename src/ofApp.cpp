#include "ofApp.h"

/*
	If you are struggling to get the device to connect ( especially Windows Users )
	please look at the ReadMe: in addons/ofxKinect/README.md
*/

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	// ofSetLogLevel(OF_LOG_SILENT);
	
	bProxyMode = true;

	if(!bProxyMode) {
		vidRecorder.setVideoCodec("prores");
		film.allocate(1024, 600, GL_RGB);
	} else {
		vidRecorder.setVideoCodec("prores");
		film.allocate(2048, 1080, GL_RGB);		
	}

	ofAddListener(vidRecorder.outputFileCompleteEvent, this, &ofApp::recordingComplete);

	film.begin();
	ofClear(255,255,255, 0);
	film.end();

	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	kinect.open();      // opens first available kinect

	bControlsOverlay = false;
	bRecording = false;
	bEncoding = false;
	bWrittingPoints = false;
	bPrevRealSize = false;

	bDrawVertices = true;
	bDrawWireframe = false;
	bDrawFaces = false;
	bViewOrbit = false;

	ofSetFrameRate(60);

	int frameTime = 1000/30;

	// zero the tilt on startup
	kinect.setCameraTiltAngle(0);

	sMeshMode = 0;
	panAngle = 0;
	tiltAngle = 0;
	pointSize = 3.0;
	stepRes = 2;
	frameNumber = 0;
	frameNumberSent = 0;

	W1.startThread();
	W2.startThread();

	previousFrameTime = 0;
	previousSavedFrameTime = 0;

	ofFileDialogResult result = ofSystemLoadDialog("Select project folder", true);
		
	if(result.bSuccess) {
		string path = result.getPath();
		ofSetDataPathRoot(path + "/");
	} 

	kinect.setLed(ofxKinect::LED_GREEN);

}

//--------------------------------------------------------------
void ofApp::update() {

	kinect.update();

	if(kinect.isFrameNew()) {
		recordFilm();
	}

	// if(W1.pointsSaver.empty()){
	// 	bWrittingPoints = false;
	// }

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(0, 0, 0);
	ofSetColor(255, 255, 255);

	if(!bRecording && (bEncoding || bWrittingPoints)){
		ofSetColor(255, 255, 0);
		ofDrawCircle(ofGetWidth() - 20, 20, 10);
	} else {
		drawFilm();
	}

	drawPreview();

	if(bControlsOverlay) {
		drawInstructions();
	}

	if(bRecording){
		ofSetColor(255, 0, 0);
		ofDrawCircle(ofGetWidth() - 20, 20, 10);
	}

	if(!bPrevRealSize) {
		ofNoFill();
		ofSetColor(255, 255, 255);
		ofDrawRectangle(270,50, ofGetWidth()-270-40, ofGetHeight()-50-40);
		ofFill();
	}

}

void ofApp::drawPreview(){
	if (bPrevRealSize) {

		int xPos = (ofGetWidth() - film.getWidth())/2;
		int yPos = (ofGetHeight() - film.getHeight())/2;

		film.draw(xPos, yPos);
	} else {
		film.draw(270,50, ofGetWidth()-270-40, ofGetHeight()-50-40);
	}

}

void ofApp::recordFilm(){
	if(bRecording){

/*		film.readToPixels(filmFrame);
		bool success = vidRecorder.addFrame(filmFrame);

		if (!success) {
			ofLogWarning("This frame was not added!");
		}*/

		if (vidRecorder.hasVideoError()) {
			ofLogWarning("The video recorder failed to write some frames!");
		}
	}

}

void ofApp::recordingComplete(ofxVideoRecorderOutputFileCompleteEventArgs& args){
	bEncoding = false;
	ofLogWarning("The recoded video file is now complete.");
}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofPointCloud pointCloud;

	pointCloud.setMode(meshMode());

	if(bReplay) {
		// TODO: Framerate needs to increase every 1/30 seconds
		long now = ofGetElapsedTimeMillis();
		if(now - previousFrameTime >= frameTime) {
			previousFrameTime = ofGetElapsedTimeMillis();
			frameNumber++;
			char fileName[20];
			sprintf(fileName,"pc-%06d.ply",frameNumber);
			string pointPath = fixPath + "/points/" + fileName;
			pointCloud.load(pointPath);
		}
	} else {
		if (stepRes < 2) stepRes = 2;
		int step = stepRes;
		for(int y = 0; y < h; y += step) {
			for(int x = 0; x < w; x += step) {
				if(kinect.getDistanceAt(x, y) > 0) {
					pointCloud.addColor(ofColor(255,255,255));
					pointCloud.addVertex(kinect.getWorldCoordinateAt(x, y));
				}
			}
		}


		if(bRecording) {
			long now = ofGetElapsedTimeMillis();
			if(now - previousFrameTime >= frameTime) {
				previousFrameTime = ofGetElapsedTimeMillis();
            	char fileName[20];
            	sprintf(fileName,"pc-%06d.ply",frameNumberSent);
            	string pointPath = pointsDirPath + fileName;
				pointCloud.setSavePath(pointPath);
				if ( frameNumberSent % 2 == 0) {
					W1.pointsSaver.send(pointCloud);
				} else {
					W2.pointsSaver.send(pointCloud);
				}
				frameNumberSent++;
			}
		} 

		if(!bEncoding && !bWrittingPoints) {
			kinect.setLed(ofxKinect::LED_GREEN);
		}
	} 


	glPointSize(pointSize);
	ofPushMatrix();
	ofRotateY(panAngle);
	ofRotateX(tiltAngle);
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit


	ofEnableDepthTest();

	if (bDrawVertices) pointCloud.drawVertices();
	if (bDrawWireframe) pointCloud.drawWireframe();
	if (bDrawFaces) pointCloud.drawFaces();

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

//--------------------------------------------------------------
void ofApp::exit() {
	ofRemoveListener(vidRecorder.outputFileCompleteEvent, this, &ofApp::recordingComplete);
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	stopRecord();
	W1.pointsSaver.close();
	W1.waitForThread(true);
	W2.pointsSaver.close();
	W2.waitForThread(true);
}

//--------------------------------------------------------------

void ofApp::startRecord() {
	if(!bEncoding && !bWrittingPoints){
		if(!vidRecorder.isInitialized()) {
			frameNumber = 0;
			frameNumberSent = 0;
			previousFrameTime = 0;
			createTakeDirectory();
			ofLogNotice() << "Recording!";
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			string videoRecordingName;
			if (bProxyMode) {
				videoRecordingName = "proxy-"+ofGetTimestampString()+".mov";
			} else {
				videoRecordingName = ofGetTimestampString()+".mov";
			}
			string videoRecordingPath = takeDirPath + videoRecordingName;
			vidRecorder.setup(videoRecordingPath, film.getWidth(), film.getHeight(), 30);
			bRecording = true;
			bEncoding = true;
		}
	} else {
	   kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
	   ofLogNotice() << "Wait! Still encoding!";
	}

	vidRecorder.start();

}

void ofApp::stopRecord() {
	if(bRecording) {
		bRecording = false;
		ofLogNotice() << "Stop Recording!";
		kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
		vidRecorder.close();
	}
}

void ofApp::createTakeDirectory(){
	ofDirectory takeDir;

	takeName = "take-" + ofGetTimestampString();

	takeDirPath = ofToDataPath(takeName + "/");

	takeDir.open(takeDirPath);

	if(!takeDir.exists()){
		takeDir.create(true);

		ofDirectory pointsDir;

		pointsDirPath = takeDirPath + "/" + "points" + "/";

		pointsDir.open(pointsDirPath);

		if(!pointsDir.exists()){
			pointsDir.create(true);
		}

		ofLogNotice() << "Take folder created!";

	}
}

void ofApp::loadTake(){
	ofFileDialogResult openFileResult = ofSystemLoadDialog("Select a take folder to replay", true);
	//Check if the user opened a file
	if (openFileResult.bSuccess){
		ofLogVerbose("User selected a file");
		fixPath = openFileResult.getPath();
		ofLogNotice() << fixPath;
		bReplay = true;

	} else {
		ofLogVerbose("User hit cancel");
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
		case 'n':
			loadTake();
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

	reportStream << " " << endl;
	if(!bRecording && (bEncoding || bWrittingPoints)) {
		int p = round(((float) frameNumber / (float) frameNumberSent)*100);
		reportStream << ">> Take: " << takeName << endl;
		reportStream << "  Wait! " << p << "%" << " encoding complete" << endl;
		reportStream << " " << endl;
	} else if (bRecording) {
		reportStream << ">> Take: " << takeName << endl;
		reportStream << "  Recording in progress!" << endl;
		reportStream << "  [s] to stop" << endl;
	} else if (!bRecording) {
		reportStream << ">> Ready!" << endl;
		reportStream << " " << endl;
		reportStream << "  [r] to record" << endl;	
	} else {
		reportStream << " " << endl;
		reportStream << " " << endl;
	}

	reportStream << " " << endl;
	reportStream << " " << endl;

	reportStream << " " << endl;
	reportStream << " " << endl;

	reportStream << "# Frames" << endl;
	reportStream << "  FPS: " << round(ofGetFrameRate()) << endl;
	reportStream << " " << endl;
	
	reportStream << "# Points" << endl;
	reportStream << "  Total: " << frameNumberSent << endl;
	reportStream << "  Pending: " << (frameNumberSent >= frameNumber ? frameNumberSent - frameNumber : 0 ) << endl;
	reportStream << "  Saved: " << frameNumber << endl;
	reportStream << " " << endl;
	
	reportStream << "# Video" << endl;
	reportStream << "  Total: " << vidRecorder.getNumVideoFramesRecorded() << endl;
	reportStream << "  Pending: " << vidRecorder.getVideoQueueSize() << endl;
	reportStream << "  Saved: " << (vidRecorder.getNumVideoFramesRecorded() >= vidRecorder.getVideoQueueSize() ? vidRecorder.getNumVideoFramesRecorded() - vidRecorder.getVideoQueueSize(): 0 ) << endl;
	reportStream << "  Mode: " << (bProxyMode?"proxy":"full") << endl;
	reportStream << "  Resolution: " << film.getWidth() << "x" << film.getHeight() << endl;
	reportStream << "  Codec: " << "Apple ProRes" << endl;
	reportStream << " " << endl;

	if(kinect.isConnected()) {
		reportStream << "# Kinect" << endl;
		reportStream << " Emitter: " << kinect.getSensorEmitterDistance() << "cm" << endl;
		reportStream << " Camera:  " << kinect.getSensorCameraDistance() << "cm" << endl;
		reportStream << " Z0 plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm" << endl;
		reportStream << " Z0 plane dist: " << kinect.getZeroPlaneDistance() << "mm" << endl;
	}

	reportStream << " " << endl;
	reportStream << " " << endl;
	reportStream << "# Controls" << endl;
	reportStream << " [r]/[s] record / stop" << endl;
	reportStream << " [p]/[l] point size: "  << pointSize << endl;
	reportStream << " [o]/[k] point size: " << stepRes << endl;
	reportStream << " [<]/[>] pan angle: " << panAngle << endl;
	reportStream << " [up]/[dn] tilt angle: " << tiltAngle << endl;
	reportStream << " [v] prev real size" << endl;
	reportStream << " [q] ensable overlay" << endl;
	reportStream << " " << endl;



	ofDrawBitmapString(reportStream.str(), 20, 20);
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

