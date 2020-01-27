#include "locodeck.h"

void setup(){

	Serial.begin(9600);

	dwm1000Init();
	Serial.print("Initialized");
	
}

void loop(){

}
