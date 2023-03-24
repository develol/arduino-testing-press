#include "HX711.h"   // HX711 v0.7.1
#include <Encoder.h> // Encoder v1.4.2

#define scaleDT  A1 // Load cells Data pin
#define scaleSCK A0 // Load cells SCK pin

#define pinStep 9 // Servo drive Step pin
#define pinDir  8 // Servo drive Direction pin

HX711   scale; // Initialization Load cells
Encoder myEnc(2, 3); // Initialization Encoder

int   buttonPinPress = 4; // Pin pressure button
int   btnPinStopUp = 11; // Pin Upper limit switch 
int   btnPinStopDwn = 10; // Pin Lower limit switch
int   stat = 0; // Current state variable
int   delta = 100; // Step between measurements
int   deltaRun = 0; // Active step between measurements
long  oldPosition  = -999; // Old position variable
long  runtime; // Execution time
long  st5Kor = 0; // 5th state variable
long  st5KorR = 0; // 5th state variable
long  st5DltKorr = 0; // 5th state variable
long  st5Ves = 0; // 5th state variable
long  st5Num = 0; // 5th state variable
long  st5Etap = 0; // 5th state variable
bool  src = true; // Active work variable
bool  direction = true; // Direction of movement
float calib = -3.7; // Calibration variable Load cells
float calibUO = 0.01; // Unit of measurement Load cells
float calibUp = 5000; // Maximum value Load cells
float U; // Variable for Get scale
float O; // Variable for Unit of measurement

// SETUP
void setup() {
	// SETUP LOAD CELLS
	scale.begin(scaleDT, scaleSCK);
	scale.set_scale();
	scale.tare();
  
	// SETUP BUTTONS
	pinMode(buttonPinPress, INPUT);
	pinMode(btnPinStopUp, INPUT);
	pinMode(btnPinStopDwn, INPUT);
  
	// SETUP SERVO DRIVE
	pinMode(pinStep, OUTPUT);
	pinMode(pinDir, OUTPUT);
  
	// SETUP SERIAL
	Serial.begin(115200);
	Serial.println("READY");
}

// SERVO DRIVE STEP
String stepper(int xw, bool fast = false) {
	if((digitalRead(btnPinStopUp))&&(direction==0)) {
		return "upStop";
	}
	if((digitalRead(btnPinStopDwn))&&(direction==1)) {
		return "downStop";
	}
	if(direction==1){ digitalWrite(pinDir, HIGH); }
	if(direction==0){ digitalWrite(pinDir, LOW); }
	if(fast){
		for(int x = 0; x < xw; x++) {
			digitalWrite(pinStep, HIGH);
			delayMicroseconds(1000);
			digitalWrite(pinStep, LOW);
			delayMicroseconds(4000);
		}      
	}else{
		for(int x = 0; x < xw; x++) {
			digitalWrite(pinStep, HIGH);
			delayMicroseconds(4000);
			digitalWrite(pinStep, LOW);
			delayMicroseconds(16000);
		}      
	}  
	return "ok";
}

// READING A VALUE FROM SERIAL
String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;
    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// OVERLOAD CHECK
bool checkFalse() {
    scale.set_scale(calib);
    U = scale.get_units();
    O = U * calibUO;    
    if(O>calibUp) {
		Serial.println("OVERLOAD");
		stat=0;
		return false;
    }
    return true;
}

// LOOP
void loop() {
	if(stat==0) { // Working mode with Serial
		String valAll = Serial.readString();
		String val0 = getValue(valAll, ' ', 0);
		if(val0=="run") {
			Serial.println("START");
			int val1 = getValue(valAll, ' ', 1).toInt();
			if(val1 == 1) {
				src = true;
				stat=1;
			}
			if(val1 == 2) {
				stat=2;
			}
			if(val1 == 3) {
				delta = getValue(valAll, ' ', 2).toInt();
				stat=5;
			}        
		}
		if(val0=="hello") {
			Serial.println("ehllo");
		}
		if(val0=="calib") {
			Serial.println("START");
			stat=3;
		}
		if(val0=="src") {
			Serial.println("START");
			stat=4;
		}
		if(val0=="up") {
			int val1 = getValue(valAll, ' ', 1).toInt();
			val1 = (val1*10)+1;
			float per = 0.1;
			Serial.println("START up");
			Serial.print("0%...");
			direction = false;
			for(int j=0; j<val1; j++) {
				if(stepper(20, true)!="ok") {break;}
					if(val1*per<j) {
						Serial.print(String((per*100),0));
						Serial.print("%...");
						per=per+0.1;
					}
					if(!checkFalse()) {break;}
			}
			Serial.println("END");   
		}
		if(val0=="down") {
			int val1 = getValue(valAll, ' ', 1).toInt();
			val1 = (val1*10)+1;
			float per = 0.1;
			Serial.println("START down");
			Serial.print("0%...");
			direction = true;
			for(int j=0; j<val1; j++) {
				if(stepper(20, true)!="ok") {break;}
				if(val1*per<j) {
					Serial.print(String((per*100),0));
					Serial.print("%...");
					per=per+0.1;
				}
				if(!checkFalse()) {break;}
			}
			Serial.println("END");   
		}
	}
	if(stat==1) { // 1st test mode
		direction = true;
		stepper(10);
		if(!digitalRead(buttonPinPress)) {
			if(src) {
				runtime = micros();
				Serial.print("Time");
				Serial.print('\t');
				Serial.print("Scale");
				Serial.print('\t');
				Serial.println("Position"); 
			}
			src = false;
			scale.set_scale(calib);
			U = scale.get_units();
			O = U * calibUO;
			Serial.print(String((micros()-runtime)/1000));
			Serial.print('\t');
			Serial.print(O);
			Serial.print('\t');
			long newPosition = myEnc.read();
			oldPosition = newPosition;
			Serial.println(newPosition);
			if(O>calibUp) {
				Serial.println("OVERLOAD");
				stat=0;
			}
		} else {
			if(!src) {
				stat=0;
				Serial.println("END");
			}
		}
	}
	if(stat==2) { // 2st test mode
		direction = true;
		stepper(10);
		if(!digitalRead(buttonPinPress)) {
			src = false; 
			Serial.print("Scale");
			Serial.print('\t');
			Serial.println("Position");           
			scale.set_scale(calib);
			U = scale.get_units();
			O = U * calibUO;
			Serial.print(O);
			Serial.print('\t');
			long newPosition = myEnc.read();
			oldPosition = newPosition;
			Serial.println(newPosition);
			if(O>calibUp){
				Serial.println("OVERLOAD");
				stat=0;
			}
		} else {
			if(!src){
				stat=0;
				Serial.println("END");
			}
		}
	}
	if(stat==3) { // Calibration mode
		scale.set_scale(calib);
		U = scale.get_units();
		O = U * calibUO;
		Serial.print(O);
		Serial.print('\t');
		long newPosition = myEnc.read();
		oldPosition = newPosition;
		Serial.print(newPosition);
		Serial.print('\t');
		Serial.print(!digitalRead(buttonPinPress));
		Serial.print('\t');
		Serial.print(digitalRead(btnPinStopUp));
		Serial.print('\t');
		Serial.println(digitalRead(btnPinStopDwn));
	}
	if(stat==4) { // Model search
		direction = true;
		stepper(10);
		if(!digitalRead(buttonPinPress)) {
			stat=0;
			direction = false;
			stepper(100);
			Serial.println("END");
		}
	}
	if(stat==5) { // 3st test mode
		direction = true;
		if(stepper(10)!="ok") {stat=0; Serial.println("END");}
		if(!digitalRead(buttonPinPress)) {
			scale.set_scale(calib);
			U = scale.get_units();
			O = U * calibUO;
			long newPosition = myEnc.read();
			oldPosition = newPosition;
			if(st5Num==0) {
				st5Num = 0;
				st5Etap = 0;
				Serial.print("Num");
				Serial.print('\t');
				Serial.print("Scale");
				Serial.print('\t');
				Serial.print("Pos");
				Serial.print('\t');
				Serial.println("DPos");
				deltaRun=O;
				st5Kor = newPosition;
				st5KorR = newPosition;
				st5Ves = O;        
			}
			if(st5Etap==0) {
				st5Num++;
				st5Etap=1;
				deltaRun=deltaRun+delta;
				st5KorR=newPosition;
				src = false;
			}
			if(deltaRun<O) {
				src = true;
				Serial.print(st5Num);
				Serial.print('\t');
				Serial.print(O - st5Ves);
				Serial.print('\t');
				Serial.print(newPosition - st5Kor);
				Serial.print('\t');
				Serial.println(st5KorR - st5Kor);
				direction = false;
				while(newPosition > st5Kor-20) {
					newPosition = myEnc.read();
					oldPosition = newPosition;
					stepper(100, true);        
				}
				st5Etap=0;
			}
			if(O>calibUp) {
				Serial.println(" ! OVERLOAD ! ");
				stat=0;
			}
		} else {
			if(!src) {
				src = true;
				scale.set_scale(calib);
				U = scale.get_units();
				O = U * calibUO;
				long newPosition = myEnc.read();
				oldPosition = newPosition;
				Serial.print("9999");
				Serial.print('\t');
				Serial.print(O - st5Ves);
				Serial.print('\t');
				Serial.print(newPosition - st5Kor);
				Serial.print('\t');
				Serial.println(st5KorR - st5Kor);
				direction = false;
				while(newPosition > st5Kor-20) {
					newPosition = myEnc.read();
					oldPosition = newPosition;
					stepper(100, true);        
				}
				delta = 100;
				deltaRun = 0;
				st5Kor = 0;
				st5KorR = 0;
				st5DltKorr = 0;
				st5Ves = 0;
				st5Num = 0;
				st5Etap = 0;
				stat=0;
				Serial.println("END");
			}
		}
	}
}