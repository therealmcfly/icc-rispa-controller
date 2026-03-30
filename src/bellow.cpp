#include <Arduino.h>
#include <Bellow.h>
#include <Wire.h>

// define function to setup the proximity sensor
void ::Bellow::setupProximitySensor()
{
	tcaPortSelect(); // select I2C port
	if (!proximitySensor.begin())
	{ // check connection to sensor
		Serial.println("Couldn't find VCNL4040 chip");
		while (1)
			;
	}
	// set sensor settings
	proximitySensor.setProximityLEDCurrent(VCNL4040_LED_CURRENT_200MA);
	proximitySensor.setProximityLEDDutyCycle(VCNL4040_LED_DUTY_1_40);
	proximitySensor.setProximityIntegrationTime(VCNL4040_PROXIMITY_INTEGRATION_TIME_8T);
	proximitySensor.setProximityHighResolution(true);
}

// define function to select the port on the I2C mulitplexer
void Bellow::tcaPortSelect()
{
	Wire.beginTransmission(TCAADDR);
	Wire.write(1 << proximitySensorPort);
	Wire.endTransmission();
}

// define function to get the proximity in mm
double Bellow::getProximity()
{
	tcaPortSelect();																																		 // select I2C port
	return (proximitySensor.getProximity() - proximityParameterB) / proximityParameterA; // readout the proximity value and convert it to mm
}

double Bellow::getProximityForICC()
{
	tcaPortSelect();																																		 // select I2C port

	double prox = (proximitySensor.getProximity() - proximityParameterB) / proximityParameterA;
	
	// ICC mapping: 16 mm is zero, then invert.
	return 16.0 - prox;
}

// define function to get the set force
double Bellow::getPressure()
{
	return pressure;
}

// define function to set the SPA pressure (p in kPa)
void Bellow::setPressure(int p)
{
	// check if p is higher than max pressure or lower than min pressure
	if (p > maxAirPressure)
	{
		p = maxAirPressure;
	}
	else if (p < minAirPressure)
	{
		p = minAirPressure;
	}

	pressure = p; // set internal variable to new value

	// check if pressure is pos or neg, set switch valve and convert pressure into PWM value
	if (p >= 0)
	{
		setSwitchValve(false);
		analogWrite(pressureValvePin, round(p / 250.0 * 255.0));
		analogWrite(vacuumValvePin, 0);
	}
	else
	{
		setSwitchValve(true);
		analogWrite(pressureValvePin, 0);
		analogWrite(vacuumValvePin, abs(round(p / 50.0 * 255.0)));
	}
}

// define function to set the enable valve (true/false)
void Bellow::setEnableValve(bool value)
{
	digitalWrite(enableValvePin, value);
}

// define function to set the switch valve (true/false)
void Bellow::setSwitchValve(bool value)
{
	digitalWrite(switchValvePin, value);
}

// define function to get the force of the force sensor
double Bellow::getForce()
{
	double force = analogRead(forceSensorPin); // readout the voltage

	// calculate the force only if a force can be measured
	if (force == 0.0)
	{
		return 0;
	}
	else
	{
		force = exp((force - fsrParameterB) / fsrParameterA);
		if (force < 0)
		{
			force = 0; // ignore negative forces
		}
	}
	return force;
}

// define function to set the valves depending on the operation mode (symmetric or asymmetric)
void Bellow::setOperationMode(mode operationMode)
{
	this->operationMode = operationMode; // set internal variable to new value
	if (operationMode == symmetric)
	{ // set the valves
		digitalWrite(enableValvePin, HIGH);
	}
	else
	{
		digitalWrite(enableValvePin, LOW);
	}
}