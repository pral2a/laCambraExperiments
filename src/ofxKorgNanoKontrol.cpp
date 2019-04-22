#include "ofxKorgNanoKontrol.h"



ofxKorgNanoKontrol::ofxKorgNanoKontrol() {
    
    
}

ofxKorgNanoKontrol::~ofxKorgNanoKontrol() {
    // clean up
    midiIn.closePort();
    midiIn.removeListener(this);
    cout << "MIDI port is closed" << endl;
}

/***********************************/
/* autoset : set true to set MIDI port auto 
   _portNum : default 0
 */
/***********************************/
void ofxKorgNanoKontrol::setup() {
    
    
    sceneButton = false;
    
    // GUI related
    ySpace = 170;
    yPosMult = 0;
    yStartPos = 220;
    xPosMult = 0;
    
    
    midiIn.openPort(0);
    
    // don't ignore sysex, timing, & active sense messages,
    // these are ignored by default
    midiIn.ignoreTypes(false, false, false);
    
    // add ofApp as a listener
    midiIn.addListener(this);
    
    // print received messages to the console
    // midiIn.setVerbose(true);
    
    
    //cout << nanoKontrolAxisMapping[0] << endl;
    
    //sliderVals.resize(nanoKontrolAxisMapping.size());
    
    
    sceneIdCounter = 1;
    
    
    for (int i = 0; i < sizeof(sliders_v1) / sizeof(int); i++) {
        
        sliderVals.resize(sliderVals.size() + 1);
        sliderVals[i].val = 0;
        sliderVals[i].midiId = sliders_v1[i];
        sliderVals[i].scene = sceneIdCounter;
        
        potVals.resize(potVals.size() + 1);
        potVals[i].val = 0;
        potVals[i].midiId = potentiometers_v1[i];
        potVals[i].scene = sceneIdCounter;
        
        
        // Hacky thing, there is no better solution
        if(i > 26) {
            sceneIdCounter++;
        }
    }
    
    // Reset scene counter for buttons
    sceneIdCounter = 1;
    // We need two sets of buttons, so set a new for loop
    for (int i = 0; i < sizeof(buttons_v1) / sizeof(int); i++) {
        buttonVals.resize(buttonVals.size() + 1);
        buttonVals[i].val = 0;
        buttonVals[i].midiId = buttons_v1[i];
        buttonVals[i].scene = sceneIdCounter;
        
        // increase the scene id every two button
        if(buttonVals[i].midiId == 17) {
            //cout  << sceneIdCounter << " midiId : " << sliderVals[i].midiId <<endl;
            sceneIdCounter++;
        }
    }
    
    
    for (int i = 0; i < sizeof(kontrol_v1) / sizeof(int); i++) {
        kontrolVals.resize(kontrolVals.size() + 1);
        kontrolVals[i].val = 0;
        kontrolVals[i].midiId = kontrol_v1[i];
        
        // SCENE button scene id = 0
        if(i == 6)
            kontrolVals[i].scene = 0;
        else
            kontrolVals[i].scene = 1;
    }
    
}

void ofxKorgNanoKontrol::newMidiMessage(ofxMidiMessage& msg) {
    
    // make a copy of the latest message
    midiMessage = msg;
    /*cout << "Received : " << ofxMidiMessage::getStatusString(midiMessage.status) << endl;
    cout << "channel: " << midiMessage.channel << endl;
    cout << "pitch: " << midiMessage.pitch << endl;
    cout << "velocity: " << midiMessage.velocity << endl;
    cout << "control: " << midiMessage.control << endl;
    cout << "value : " << msg.value << endl;
    cout << "delta: " << midiMessage.deltatime << endl;*/
    
    
    // Slider Handler
    for (int i = 0; i < sliderVals.size(); i++) {
        
        if(midiMessage.channel == sliderVals[i].scene && midiMessage.control == sliderVals[i].midiId) {
            sliderVals[i].val = midiMessage.value;
            ofNotifyEvent(sliderValChanged, midiMessage.value, this);
        }
        
        
        if(midiMessage.channel == potVals[i].scene && midiMessage.control == potVals[i].midiId) {
            potVals[i].val = midiMessage.value;
            ofNotifyEvent(potValChanged, midiMessage.value, this);
        }
    }
    // Button Handler
    for (int i = 0; i < buttonVals.size(); i++) {
        if(midiMessage.channel == buttonVals[i].scene && midiMessage.control == buttonVals[i].midiId) {
            buttonVals[i].val = midiMessage.value;
            ofNotifyEvent(pushButtonPressed, midiMessage.value, this);
            //cout << buttonVals[i].val << " : " << buttonVals[i].scene << " : " << buttonVals[i].midiId << " : " << i << endl;
        }
    }
    
    
    // Addional Handler
    for (int i = 0; i < kontrolVals.size(); i++) {
        if(midiMessage.channel == kontrolVals[i].scene && midiMessage.control == kontrolVals[i].midiId) {
            kontrolVals[i].val = midiMessage.value;
            //
            // The last one is the SCENE button on KORG Nano Kontrol. It sends 0, so set it manually to get message
            if(i == 6 && midiMessage.value == 0) {
                ofNotifyEvent(sceneButtonPressed, midiMessage.value, this);
                //sceneButton = !sceneButton;
                //kontrolVals[i].val = sceneButton;
            }else{
                //sceneButton = !sceneButton;
                //kontrolVals[i].val = sceneButton;
            }
            //cout << kontrolVals[i].val << " : " << kontrolVals[i].scene << " : " << kontrolVals[i].midiId << " : " << i << endl;
        }
    }
    
    //cout << midiMessage.status << " " << midiMessage.control << " " << midiMessage.channel  << endl;
    //cout << sliderVals[0].val << midiMessage.control << " : " << midiMessage.channel << " : " << midiMessage.value << endl;
}

int ofxKorgNanoKontrol::getVal(int _control,int _type, int _sceneId) {
    //cout << "value : " << sliderVals[_control].val << endl;
    
    
    // Todo: Need to edit this condition for buttons
    if(_sceneId > 1) {
        if(_sceneId == 2) {
            _control = 9 + _control;
        }
    
        if(_sceneId == 3) {
            _control = 18 + _control;
        }
        
        if(_sceneId == 4) {
            _control = 27 + _control;
        }
    
    }
    
    if(_type == K_TYPE_SLIDER)
        return sliderVals[_control].val;
    
    if(_type == K_TYPE_POT)
        return potVals[_control].val;
    
    if(_type == K_TYPE_BUTTON)
        return buttonVals[_control].val;
    
    if(_type == K_TYPE_MENU_BUTTONS)
        return kontrolVals[_control].val;
}

int ofxKorgNanoKontrol::getSliderVal(int _control,int _sceneId) {
    
}

int ofxKorgNanoKontrol::getPotVal(int _control,int _sceneId) {
    
}

int ofxKorgNanoKontrol::getButtonVal(int _control,int _sceneId) {
    
}
/*
 void ofxKorgNanoKontrol::exit() {
 cout << "MIDI port is closed via exit() function" << endl;
 }*/