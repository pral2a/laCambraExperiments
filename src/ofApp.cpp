#include "ofApp.h"

/*
	If you are struggling to get the device to connect ( especially Windows Users )
	please look at the ReadMe: in addons/ofxKinect/README.md
*/

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	//ofSetLogLevel(OF_LOG_SILENT);
	
	bProxyMode = false;

	pointCloud.setMode(meshMode());
	pointCloudPreLoaded.setMode(meshMode());

	// https://wideopenbokeh.com/AthenasFall/?p=111
	// https://github.com/timscaffidi/ofxVideoRecorder/issues/47
	
	if(!bProxyMode) {
		// vidRecorder.setVideoCodec("prores");
		film.allocate(1920, 1080, GL_RGBA);
	} else {
		// vidRecorder.setVideoCodec("prores");
		film.allocate(1024, 600, GL_RGBA);
	}

	vidRecorder.setVideoCodec("prores_ks");
	vidRecorder.setVideoBitrate("8000k");
	vidRecorder.setPixelFormat("rgba");
	vidRecorder.setOutputPixelFormat("rgba");

	ofAddListener(vidRecorder.outputFileCompleteEvent, this, &ofApp::recordingComplete);

	film.begin();
	ofClear(255,255,255, 0);
	film.end();

	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();

	kinect.open();      // opens first available kinect

	bControlsOverlay = false;
	bRecording = false;
	bEncoding = false;
	bWrittingPoints = false;
	bPrevRealSize = false;
	bProjectionOrtho = false;

	bDrawVertices = true;
	bDrawWireframe = false;
	bDrawFaces = false;
	bViewOrbit = false;
	bReplayPause = true;
	bNewFrame = false;

	//ofSetFrameRate(60);


	playbackFrameRate = defaultFrameRate;

	// zero the tilt on startup
	kinect.setCameraTiltAngle(0);

	sMeshMode = 0;
	panAngle = 0;
	tiltAngle = 0;
	pointSize = 3.0;
	stepRes = 2;
	frameNumber = 0;
	frameLoaded = 0;
	frameNumberSent = 0;
	playbackFrameNumber = 0;

	for(int i = 0; i < totalWorkers; i++){
		ofSaveWorker W[i];
	}

	for(int i = 0; i < totalWorkers; i++){
		W[i].startThread();
	}
    	
	previousFrameTime = 0;
	previousSavedFrameTime = 0;
	previousFolderCheckTime = 0;
	previousEnlapsedFrameTime = frameTime;
	frameCompensation = 0;

	ofFileDialogResult result = ofSystemLoadDialog("Select project folder", true);
		
	if(result.bSuccess) {
		string path = result.getPath();
		ofSetDataPathRoot(path + "/");
	} 

	kinect.setLed(ofxKinect::LED_GREEN);

	frameThing = 0;

	easyCam.setAspectRatio(film.getWidth()/film.getHeight());

	if (!bProjectionOrtho){
		easyCam.disableOrtho();
		easyCam.setFov(100);
	}

	easyCam.enableMouseMiddleButton();

	bKontrol = true;

    nano.setup();

    camSpeed = 0.1;

    

}

//--------------------------------------------------------------
void ofApp::update() {

	if (bKontrol) {
		easyCam.setDistance(ofMap(nano.getVal(K_SLIDER_3),0,127,0,1000));
	}

	easyCam.setFov(ofMap(nano.getVal(K_SLIDER_6),0,127, 28, 200));

	pointSize = ofMap(nano.getVal(K_SLIDER_7),0,127,0,30);


	// kinect.update();

	// if(kinect.isFrameNew()) {
	// 	recordFilm();
	// }


	if(bReplay) {
		recordFilm();
	}

	if(!bRecording && bWrittingPoints){
		long now = ofGetElapsedTimeMillis();
		if(now - previousFolderCheckTime >= 3000){
			previousFolderCheckTime = ofGetElapsedTimeMillis();
			ofDirectory pointsDir;
	    	ofLogNotice() << "pointPath" << pointsDirPath;
	   		pointsDir.open(pointsDirPath);
	    	int numFiles = pointsDir.listDir();
	    	frameNumber = numFiles;
	    	ofLogNotice() << "frameNumberFolder" << numFiles;
	    	if(frameNumber >= frameNumberSent){
	    		bWrittingPoints = false;
	    	}
		}
	
	}



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
	bEncoding = false;
	ofLogWarning("The recoded video file is now complete.");
}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;

	frameThing++;

	if(bReplay) {

		if (playbackFrameNumber == frameLoaded) {
			int nextFrame = playbackFrameNumber + 1;
			char fileName[20];
			sprintf(fileName,"pc-%06d.ply", nextFrame);
			string pointPath = fixPath + "/points/" + fileName;
		    ofxBinaryMesh::load(pointPath, pointCloudPreLoaded);
		    frameLoaded = nextFrame;
		} 


		if(frameThing >= round(ofGetFrameRate()) / playbackFrameRate) {
			frameThing = 0;

			if (!bReplayPause) {
				if (playbackFrameNumber < totalPlaybackFrameNumber) {
					playbackFrameNumber++;
				}
			}
			pointCloud = pointCloudPreLoaded;
		}


	   float rand = ofMap(nano.getVal(K_SLIDER_5),0,127,0,100);
	   float t = ofMap(nano.getVal(K_SLIDER_4),0,127,0,2000);


	  	for(int j=0; j<pointCloud.getNumVertices(); j++){

	  		//float r = ofRandom(-t,  t);

	  		float r = ofRandom(-rand,  rand);

	     	pointCloud.setVertex(j, pointCloud.getVertex(j) + ofVec3f(r,r,r));

	  		if (t < 2000 && pointCloud.getVertex(j)[2] > t) {
	  			pointCloud.setVertex(j, ofVec3f(0,0,0));
	  		}

	     	// pointCloud.setVertex(j, pointCloud.getVertex(j) + ofVec3f(r,r,r));

	    	// float r = ofRandom(-100,  100);


	  	}



		//int q = 0;

		//ofLogWarning() << pointCloud.getNumVertices();

	  	// for(int j=0; j<pointCloud.getNumVertices(); j++){


	  	// 	if (q < round(ofRandom(10, 20))) {
	  	// 		q++;
	  	// 		float r = 0;
	   //  		pointCloud.setVertex(j, ofVec3f(r,r,r));
	  	// 	} else {
	  	// 		q = 0;
	  	// 	}


	  	// }


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
			long now = ofGetElapsedTimeMicros();
			if(now - previousFrameTime >= frameTime) {
				previousFrameTime = ofGetElapsedTimeMicros();
				bWrittingPoints = true;
            	char fileName[20];
            	sprintf(fileName,"pc-%06d.ply",frameNumberSent);
            	string pointPath = pointsDirPath + fileName;
				pointCloud.setSavePath(pointPath);
				int worker = frameNumberSent % totalWorkers;
				W[worker].pointsSaver.send(pointCloud);
				frameNumberSent++;
			}
		} 

		if(!bEncoding && !bWrittingPoints) {
			kinect.setLed(ofxKinect::LED_GREEN);
		}
	} 

	// Some fun!
	// pointSize = ofRandom(pointSize -  0.1f,  pointSize + 0.1f);
	glPointSize(pointSize);
	ofPushMatrix();

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
	ofClear(0,0,0,0);  	// ofBackground(0, 0, 0);
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
	for(int i = 0; i < totalWorkers; i++){
		W[i].pointsSaver.close();
		W[i].waitForThread(true);
	}
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
			string videoRecordingPath;
			if (bReplay) {
				videoRecordingName = "export-"+ofGetTimestampString()+".mov";
				videoRecordingPath = fixPath + '/' + videoRecordingName;
		    } else {
				if (bProxyMode) {
					videoRecordingName = "proxy-"+ofGetTimestampString()+".mov";
				} else {
					videoRecordingName = ofGetTimestampString()+".mov";
				}
				videoRecordingPath = takeDirPath + videoRecordingName;
		    }
			vidRecorder.setup(videoRecordingPath, film.getWidth(), film.getHeight(), 25);
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
		
		ofDirectory pointsDir;
	   	pointsDir.open(fixPath + "/" + "points");
	  
	    totalPlaybackFrameNumber = pointsDir.listDir();

		ofLogNotice() << fixPath;
		bReplay = true;
		playbackFrameNumber = 0;

	} else {
		ofLogVerbose("User hit cancel");
	}


}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bReplayPause = !bReplayPause;
			break;
		case 'x':
			if(bReplayPause){
				if (playbackFrameNumber + 30 <= totalPlaybackFrameNumber) {
					playbackFrameNumber = playbackFrameNumber + 30;
					frameLoaded = playbackFrameNumber;
				}
			}
			break;
		case 'z':
			if(bReplayPause){
				if (playbackFrameNumber > 30) {
					playbackFrameNumber = playbackFrameNumber - 30;
					frameLoaded = playbackFrameNumber;
				}
			}
			break;
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
			bKontrol = !bKontrol;
			//stepRes--;
			break;
		case 'f':
			playbackFrameRate = defaultFrameRate;
			break;
		case 'd':
			bProjectionOrtho = !bProjectionOrtho;
			if (bProjectionOrtho) {
				easyCam.enableOrtho();
			} else {
				easyCam.disableOrtho();
				easyCam.setFov(100);
			}
			break;
		case 'n':
			loadTake();
			break;
		case 'h':
			playbackFrameNumber = 0;
			frameLoaded = playbackFrameNumber;
			break;
		case 'g':
			sMeshMode = 0;
			pointSize = 3.0;
			stepRes = 2;
			easyCam.reset();
			easyCam.setFov(100);
			break;
		case OF_KEY_RIGHT:
			easyCam.setPosition(easyCam.getX() + camSpeed, easyCam.getY(), easyCam.getZ());
			break;
		case OF_KEY_LEFT:
			easyCam.setPosition(easyCam.getX() - camSpeed, easyCam.getY(), easyCam.getZ());
			break;
		case OF_KEY_UP:
			easyCam.setPosition(easyCam.getX(), easyCam.getY() + camSpeed, easyCam.getZ());
			break;
		case OF_KEY_DOWN:
			easyCam.setPosition(easyCam.getX(), easyCam.getY() - camSpeed, easyCam.getZ());
			break;
		}
}

//--------------------------------------------------------------
void ofApp::drawInstructions() {
		// draw instructions
	ofSetColor(255, 255, 255);

	stringstream reportStream;

	reportStream << " " << endl;

	int vidSaved = (vidRecorder.getNumVideoFramesRecorded() >= vidRecorder.getVideoQueueSize() ? vidRecorder.getNumVideoFramesRecorded() - vidRecorder.getVideoQueueSize(): 0 );

	if(!bRecording && (bEncoding || bWrittingPoints)) {

		int remainingPercent = round(((float) frameNumber / (float) frameNumberSent)*100);

		if (remainingPercent < 0) {
			remainingPercent = round(((float) vidSaved / (float) vidRecorder.getNumVideoFramesRecorded())*100);
		}

		reportStream << ">> Take: " << takeName << endl;
		reportStream << "  Wait! " << remainingPercent << "%" << " encoding complete" << endl;
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

	reportStream << "# Perf" << endl;
	reportStream << "  FPS: " << round(ofGetFrameRate()) << endl;

	if (bReplay) {
		float playbackPerf = (float) ofGetFrameRate() / playbackFrameRate;
		reportStream << "  P: " << round (playbackPerf * 100) << " %"  << endl;
		if (playbackPerf < 1) {
			reportStream << "  Jitter!" << endl;
		} else {
			reportStream << "  " << endl;
		}
	}

	reportStream << " " << endl;

	if (bReplay) {
	
		reportStream << "# Points Play" << endl;
		reportStream << "  FPS: " << playbackFrameRate << endl;
		reportStream << "  Frame: " << playbackFrameNumber << endl;
		reportStream << "  Pending: " << totalPlaybackFrameNumber - playbackFrameNumber << endl;
		reportStream << "  Total: " << totalPlaybackFrameNumber << endl;
		reportStream << " " << endl;

    } else {

		reportStream << "# Points Rec" << endl;
		reportStream << "  Total: " << frameNumberSent << endl;
		reportStream << "  Pending: " << (frameNumberSent >= frameNumber ? frameNumberSent - frameNumber : 0 ) << endl;
		reportStream << "  Saved: " << frameNumber << endl;
		reportStream << " " << endl;

    }
	
	reportStream << "# Video" << endl;
	reportStream << "  Total: " << vidRecorder.getNumVideoFramesRecorded() << endl;
	reportStream << "  Pending: " << vidRecorder.getVideoQueueSize() << endl;
	reportStream << "  Saved: " << vidSaved << endl;
	reportStream << "  Mode: " << (bProxyMode?"proxy":"full") << endl;
	reportStream << "  Resolution: " << film.getWidth() << "x" << film.getHeight() << endl;
	reportStream << "  Codec: " << "Apple ProRes" << endl;
	reportStream << " " << endl;

	reportStream << "# Cam" << endl;
	reportStream << "  Ratio: " << easyCam.getAspectRatio() << endl;
	reportStream << "  Lens: " << easyCam.getFov() << endl;
	reportStream << "  Zoom: " << easyCam.getDistance() << endl;
	reportStream << "  X: " << easyCam.getX() << endl;
	reportStream << "  Y: " << easyCam.getY() << endl;
	reportStream << "  Z: " << easyCam.getZ() << endl;
	reportStream << " " << endl;

/*	if(kinect.isConnected()) {
		reportStream << "# Kinect" << endl;
		reportStream << " Emitter: " << kinect.getSensorEmitterDistance() << "cm" << endl;
		reportStream << " Camera:  " << kinect.getSensorCameraDistance() << "cm" << endl;
		reportStream << " Z0 plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm" << endl;
		reportStream << " Z0 plane dist: " << kinect.getZeroPlaneDistance() << "mm" << endl;
	}*/

	reportStream << " " << endl;
	reportStream << " " << endl;
	reportStream << "# Controls" << endl;

	reportStream << " [r]/[s] record / stop" << endl;

	reportStream << " [r]/[s] record / stop" << endl;
	reportStream << " [p]/[l] point size: "  << pointSize << endl;
	reportStream << " [o]/[c] point size: " << stepRes << endl;
	reportStream << " [<]/[>] pan angle: " << panAngle << endl;
	reportStream << " [up]/[dn] tilt angle: " << tiltAngle << endl;
	reportStream << " [v] prev real size" << endl;
	reportStream << " [q] overlay ctrls" << endl;
	reportStream << " [g] reset view" << endl;
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

