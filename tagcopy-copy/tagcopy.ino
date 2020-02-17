#include "locodeck.h"
#include <SPI.h>

void setup(){

	Serial.begin(9600);
	// SPI.begin();
	dwm1000Init();
	Serial.print("Initialized");
	
}

void loop(){

}
