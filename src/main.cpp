#include <Arduino.h>
#include <Wire.h>		// library for I2C control
#include <Bellow.h> // include header for bellow control

// declare and initialise variables
#define waitTimeSymmetric 100			 // ms (symmetric mode)
#define waitTimePipePressurised 60 // ms (asymmetric mode)
#define waitTimeNextBellow 140		 // ms (asymmetric mode)
#define buttonPin 3
#define PI 3.1415926535897932384626433832795
mode operationMode = off;
// mode operationMode = icc;
uint32_t buttonOldTime = millis();
uint8_t iccRxBytes[2];
uint8_t iccRxPos = 0;
mode lastReportedMode = off;

// #define ICC1D_PACKET_DEBUG 1

// instantiation of the class Bellow (create object)
// Bellow bn(proximitySensorPort, forceSensorPin, switchValvePin, enableValvePin, proximityParameterA, proximityParameterB, fsrParameterA, fsrParameterB);
// Bellow b1(7, A4, 22, 24, 64.624, 1415.8, 250.34, 353.6);
// Bellow b2(6, A3, 26, 28, 41.764, 1866.7, 200.86, 310.22);
// Bellow b3(5, A2, 30, 32, 33.939, 1194.8, 205.93, 229.06);
// Bellow b4(4, A1, 34, 36, 38.048, 2171.7, 171.6, 325.88);
// Bellow b5(3, A0, 38, 40, 37.509, 1788, 258.52, 372.67);
Bellow b1(7, A4, 22, 24, 64.624, 1415.8, 250.34, 353.6);
Bellow b2(6, A3, 26, 28, 41.764, 1866.7, 200.86, 310.22);
Bellow b3(5, A2, 30, 32, 33.939, 1194.8, 205.93, 229.06);
Bellow b4(4, A1, 34, 36, 38.048, 2171.7, 171.6, 325.88);
Bellow b5(3, A0, 38, 40, 37.509, 1788, 258.52, 372.67);

// function to set operation modes of all five SPA
void setAllOperationModes(mode operationMode)
{
	b1.setOperationMode(operationMode);
	b2.setOperationMode(operationMode);
	b3.setOperationMode(operationMode);
	b4.setOperationMode(operationMode);
	b5.setOperationMode(operationMode);
}

// function to set a new pressure for all five SPA
void setAllPressures(int pressure)
{
	b1.setPressure(pressure);
	b2.setPressure(pressure);
	b3.setPressure(pressure);
	b4.setPressure(pressure);
	b5.setPressure(pressure);
}

// function to activate only the required enable valve
void setEnableValves(int bellowNumber)
{
	switch (bellowNumber)
	{
	case 0:
		b1.setEnableValve(false);
		b2.setEnableValve(false);
		b3.setEnableValve(false);
		b4.setEnableValve(false);
		b5.setEnableValve(false);
		break;

	case 1:
		b1.setEnableValve(true);
		b2.setEnableValve(false);
		b3.setEnableValve(false);
		b4.setEnableValve(false);
		b5.setEnableValve(false);
		break;

	case 2:
		b1.setEnableValve(false);
		b2.setEnableValve(true);
		b3.setEnableValve(false);
		b4.setEnableValve(false);
		b5.setEnableValve(false);
		break;

	case 3:
		b1.setEnableValve(false);
		b2.setEnableValve(false);
		b3.setEnableValve(true);
		b4.setEnableValve(false);
		b5.setEnableValve(false);
		break;

	case 4:
		b1.setEnableValve(false);
		b2.setEnableValve(false);
		b3.setEnableValve(false);
		b4.setEnableValve(true);
		b5.setEnableValve(false);
		break;

	case 5:
		b1.setEnableValve(false);
		b2.setEnableValve(false);
		b3.setEnableValve(false);
		b4.setEnableValve(false);
		b5.setEnableValve(true);
		break;
	case 6:
		b1.setEnableValve(true);
		b2.setEnableValve(true);
		b3.setEnableValve(true);
		b4.setEnableValve(true);
		b5.setEnableValve(true);
		break;
	}
}

// function to handle the interrupt when button pressed
void buttonInterrupt()
{
	uint32_t buttonTimePressed = millis() - buttonOldTime; // calculate how long the button was pressed
	if ((buttonTimePressed > 50) && (buttonTimePressed < 250))
	{ // press-time must be >50 ms to avoid bouncing
		// change operation mode to next state
		// switch (operationMode)
		// {
		// case (off):
		// 	operationMode = symmetric;
		// 	break;
		// case (symmetric):
		// 	operationMode = asymmetric;
		// 	break;
		// case (asymmetric):
		// 	operationMode = icc;
		// 	break;
		// case (icc):
		// 	operationMode = icc1d;
		// 	break;
		// case (icc1d):
		// 	operationMode = off;
		// 	break;
		// }
		switch (operationMode)
		{
		case (off):
			operationMode = icc1d;
			break;
		case (icc1d):
			operationMode = off;
			break;
		}
	}
	buttonOldTime = millis(); // save the current time for the next button-press-time calculation
}

// function to readout all sensor values
void printData()
{
	Serial.println("Bellow 1:");
	Serial.print("p = ");
	Serial.print(b1.getPressure());
	Serial.println(" kPa");
	Serial.print("F = ");
	Serial.print(b1.getForce());
	Serial.println(" N");
	Serial.print("x = ");
	Serial.print(b1.getProximity());
	Serial.println(" mm");
	Serial.println("Bellow 2:");
	Serial.print("p = ");
	Serial.print(b2.getPressure());
	Serial.println(" kPa");
	Serial.print("F = ");
	Serial.print(b2.getForce());
	Serial.println(" N");
	Serial.print("x = ");
	Serial.print(b2.getProximity());
	Serial.println(" mm");
	Serial.println("Bellow 3:");
	Serial.print("p = ");
	Serial.print(b3.getPressure());
	Serial.println(" kPa");
	Serial.print("F = ");
	Serial.print(b3.getForce());
	Serial.println(" N");
	Serial.print("x = ");
	Serial.print(b3.getProximity());
	Serial.println(" mm");
	Serial.println("Bellow 4:");
	Serial.print("p = ");
	Serial.print(b4.getPressure());
	Serial.println(" kPa");
	Serial.print("F = ");
	Serial.print(b4.getForce());
	Serial.println(" N");
	Serial.print("x = ");
	Serial.print(b4.getProximity());
	Serial.println(" mm");
	Serial.println("Bellow 5:");
	Serial.print("p = ");
	Serial.print(b5.getPressure());
	Serial.println(" kPa");
	Serial.print("F = ");
	Serial.print(b5.getForce());
	Serial.println(" N");
	Serial.print("x = ");
	Serial.print(b5.getProximity());
	Serial.println(" mm");
	Serial.println();
}

// read one binary big-endian int16 pressure command from Serial1 (non-blocking)
bool tryReadIccPressure(int *pressureOut)
{
	while (Serial1.available() > 0)
	{
		uint8_t rxByte = (uint8_t)Serial1.read();
		iccRxBytes[iccRxPos++] = rxByte;

		// Serial.print("ICC RX byte[");
		// Serial.print(iccRxPos - 1);
		// Serial.print("]=0x");
		// if (rxByte < 0x10)
		// {
		// 	Serial.print("0");
		// }
		// Serial.println(rxByte, HEX);

		if (iccRxPos == 2)
		{
			// ICC data arrives on Serial1 as big-endian int16: byte[0] is MSB, byte[1] is LSB.
			*pressureOut = (int)(int16_t)((uint16_t)iccRxBytes[0] | ((uint16_t)iccRxBytes[1] << 8));

			// Serial.print("ICC RX packet: [0x");
			// if (iccRxBytes[0] < 0x10)
			// {
			// 	Serial.print("0");
			// }
			// Serial.print(iccRxBytes[0], HEX);
			// Serial.print(", 0x");
			// if (iccRxBytes[1] < 0x10)
			// {
			// 	Serial.print("0");
			// }
			// Serial.print(iccRxBytes[1], HEX);
			// Serial.print("] => ");
			// Serial.println(*pressureOut);

			iccRxPos = 0;

			double prox = b1.getProximity();
			// double prox = 5.0f; // for testing
			// Serial.println(prox);
			Serial1.write((uint8_t *)&prox, sizeof(double));
			return true;
		}
	}
	return false;
}

bool tryReadIccPressureVec(int16_t pressureOut[5])
{
	static const uint8_t SOF0 = 0xAA;
	static const uint8_t SOF1 = 0x55;
	static uint8_t payload[10];			// 5 * int16
	static uint8_t parserState = 0; // 0: wait SOF0, 1: wait SOF1, 2: read payload
	static uint8_t payloadPos = 0;
	static uint32_t icc1dPacketCount = 0;

	while (Serial1.available() > 0)
	{
		uint8_t rxByte = (uint8_t)Serial1.read();

		if (parserState == 0)
		{
			if (rxByte == SOF0)
			{
				parserState = 1;
			}
			continue;
		}

		if (parserState == 1)
		{
			if (rxByte == SOF1)
			{
				parserState = 2;
				payloadPos = 0;
			}
			else if (rxByte == SOF0)
			{
				parserState = 1;
			}
			else
			{
				parserState = 0;
			}
			continue;
		}

		payload[payloadPos++] = rxByte;

		if (payloadPos == 10)
		{
			icc1dPacketCount++;

#ifdef ICC1D_PACKET_DEBUG
			int16_t leVec[5];
			int16_t beVec[5];
			for (uint8_t i = 0; i < 5; i++)
			{
				uint8_t b0 = payload[i * 2];
				uint8_t b1 = payload[i * 2 + 1];
				leVec[i] = (int16_t)(((uint16_t)b0) | ((uint16_t)b1 << 8));
				beVec[i] = (int16_t)(((uint16_t)b0 << 8) | (uint16_t)b1);
			}

			if (icc1dPacketCount <= 20 || (icc1dPacketCount % 50) == 0)
			{
				Serial.print("ICC1D PKT #");
				Serial.println(icc1dPacketCount);

				Serial.print("raw payload bytes: ");
				for (uint8_t i = 0; i < 10; i++)
				{
					if (payload[i] < 0x10)
					{
						Serial.print("0");
					}
					Serial.print(payload[i], HEX);
					if (i < 9)
					{
						Serial.print(" ");
					}
				}
				Serial.println();

				Serial.print("decode LE [b0|b1<<8]: ");
				for (uint8_t i = 0; i < 5; i++)
				{
					Serial.print(leVec[i]);
					if (i < 4)
					{
						Serial.print(", ");
					}
				}
				Serial.println();

				Serial.print("decode BE [b0<<8|b1]: ");
				for (uint8_t i = 0; i < 5; i++)
				{
					Serial.print(beVec[i]);
					if (i < 4)
					{
						Serial.print(", ");
					}
				}
				Serial.println();
			}
#endif

			// Decode incoming 5x int16, little-endian per cell [LSB][MSB]
			for (uint8_t i = 0; i < 5; i++)
			{
				uint8_t lsb = payload[i * 2];
				uint8_t msb = payload[i * 2 + 1];
				pressureOut[i] = (int16_t)(((uint16_t)lsb) | ((uint16_t)msb << 8));
			}

			Serial.print("ICC1D RX kPa: ");
			for (uint8_t i = 0; i < 5; i++)
			{
				Serial.print(pressureOut[i]);
				if (i < 4)
				{
					Serial.print(", ");
				}
			}
			Serial.println();

			parserState = 0;
			payloadPos = 0;

			// // test: send back proximity vector with header
			// uint8_t rxHeader[2] = {0xAA, 0x55};
			// double mockProx = 5.0f; // for testing
			// Serial1.write(rxHeader, sizeof(rxHeader));
			// // Serial.println(prox);
			// Serial1.write((uint8_t *)&mockProx, sizeof(double));

			return true;
		}
	}

	return false;
}

const char *modeToString(mode m)
{
	switch (m)
	{
	case off:
		return "off";
	case symmetric:
		return "symmetric";
	case asymmetric:
		return "asymmetric";
	case icc:
		return "icc";
	case icc1d:
		return "icc1d";
	default:
		return "unknown";
	}
}

// setup function
void setup()
{

	// setup interrupt for push button S1
	pinMode(buttonPin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(buttonPin), buttonInterrupt, CHANGE);

	// setup I2C
	Wire.begin();
	Serial.begin(115200);
	Serial1.begin(115200);
	while (!Serial)
	{
		delay(1); // wait until serial port is opened
	}

	Serial.println("USB Serial ready; ICC data on Serial1");

	// setup PWM Speed
	TCCR3A = 0b00000001;
	TCCR3B = 0b00001001;

	// call all proximity sensor setup functions
	b1.setupProximitySensor();
	b2.setupProximitySensor();
	b3.setupProximitySensor();
	b4.setupProximitySensor();
	b5.setupProximitySensor();

	// activate all enable valves (initial position)
	b1.setEnableValve(true);
	b2.setEnableValve(true);
	b3.setEnableValve(true);
	b4.setEnableValve(true);
	b5.setEnableValve(true);
}

// main loop function
void loop()
{
	if (operationMode != lastReportedMode)
	{
		Serial.print("Mode changed to: ");
		Serial.println(modeToString(operationMode));
		lastReportedMode = operationMode;
	}

	// check current operation mode
	switch (operationMode)
	{
	case off:
		// initial position
		setAllOperationModes(symmetric);
		setAllPressures(0);
		Serial.print("");
		break;

	case symmetric:
		setAllOperationModes(operationMode);
		for (int i = 0; i < 200; i++)
		{
			if (operationMode == symmetric)
			{																													 // check if button was pressed during for loop
				setAllPressures(round(65 * sin(2 * PI / 200 * i) + 15)); // set new air pressure to pessure valves
				delay(waitTimeSymmetric);																 // waiting time until next pressure is set
				Serial.print("");
			}
		}
		break;

	case asymmetric:
		setAllOperationModes(operationMode);
		for (int i = 0; i < 50; i++)
		{
			if (operationMode == asymmetric)
			{																												// check if button was pressed during for loop
				setEnableValves(0);																		// close all enable valves
				b1.setPressure(round(45 * sin(2 * PI / 50 * i) - 5)); // set new air pressure to pessure valves
				delay(waitTimePipePressurised);												// waiting time until the new air pressure has set in the pipes
				setEnableValves(1);																		// open enable valve of bellow 1
				delay(waitTimeNextBellow);														// waiting time until the new air pressure has set in the bellow
				setEnableValves(0);																		// close all enable valves
				b2.setPressure(round(55 * sin(2 * PI / 50 * i) + 5)); // ...
				delay(waitTimePipePressurised);
				setEnableValves(2);
				delay(waitTimeNextBellow);
				setEnableValves(0);
				b3.setPressure(round(40 * sin(2 * PI / 50 * i) - 10));
				delay(waitTimePipePressurised);
				setEnableValves(3);
				delay(waitTimeNextBellow);
				setEnableValves(0);
				b4.setPressure(round(69 * sin(2 * PI / 50 * i) + 17));
				delay(waitTimePipePressurised);
				setEnableValves(4);
				delay(waitTimeNextBellow);
				setEnableValves(0);
				b5.setPressure(round(65 * sin(2 * PI / 50 * i) + 15));
				delay(waitTimePipePressurised);
				setEnableValves(5);
				delay(waitTimeNextBellow);
				Serial.print("");
			}
		}
		break;

		// case icc:
		// 	Serial.println("Entered ICC mode: awaiting pressure commands on Serial1");
		// 	setAllOperationModes(symmetric);
		// 	setAllPressures(0);
		// 	delay(1000); // let any late USB CDC bytes arrive
		// 	setEnableValves(0);

		// 	// Resync packet parsing on ICC entry in case any stale startup bytes are pending.
		// 	iccRxPos = 0;

		// 	// flush
		// 	unsigned int flushedBytes = 0;
		// 	while (Serial1.available() > 0)
		// 	{
		// 		Serial1.read();
		// 		flushedBytes++;
		// 	}
		// 	delay(50); // let any late USB CDC bytes arrive

		// 	setEnableValves(1);

		// 	while (Serial1.available() > 0)
		// 	{
		// 		Serial1.read();
		// 		flushedBytes++;
		// 	}
		// 	Serial.print("ICC mode: startup RX flush complete, discarded bytes=");
		// 	Serial.println(flushedBytes);

		// 	int pressureCmd = 0;
		// 	int prevPressureCmd = 0;
		// 	while (operationMode == icc)
		// 	{
		// 		if (tryReadIccPressure(&pressureCmd))
		// 		{
		// 			// Serial.println(pressureCmd);
		// 			if (prevPressureCmd != pressureCmd)
		// 			{
		// 				b1.setPressure(pressureCmd);
		// 				// setAllPressures(pressureCmd);

		// 				prevPressureCmd = pressureCmd;
		// 			}
		// 		}
		// 	}
		// 	setEnableValves(0); // close all enable valves
		// 	break;

	case icc1d:
		Serial.println("Entered ICC1D mode: awaiting 5-cell pressure vector on Serial1");
		setAllOperationModes(symmetric);
		setAllPressures(0);
		delay(1000); // let any late startup bytes arrive
		setEnableValves(0);

		// Resync packet parsing on ICC_1D entry in case stale bytes are pending.
		iccRxPos = 0;

		// flush
		unsigned int flushedVecBytes = 0;
		while (Serial1.available() > 0)
		{
			Serial1.read();
			flushedVecBytes++;
		}
		delay(50);

		setEnableValves(0); // open all enable valves

		while (Serial1.available() > 0)
		{
			Serial1.read();
			flushedVecBytes++;
		}
		Serial.print("ICC_1D mode: startup RX flush complete, discarded bytes=");
		Serial.println(flushedVecBytes);

		int16_t pressureCmdVec[5] = {0, 0, 0, 0, 0};
		int16_t prevPressureCmdVec[5] = {0, 0, 0, 0, 0};
		bool havePrevVec = false;

		while (operationMode == icc1d)
		{
			if (tryReadIccPressureVec(pressureCmdVec))
			{
				// bool changed = !havePrevVec;
				// if (!changed)
				// {
				// 	for (uint8_t i = 0; i < 5; i++)
				// 	{
				// 		if (pressureCmdVec[i] != prevPressureCmdVec[i])
				// 		{
				// 			changed = true;
				// 			break;
				// 		}
				// 	}
				// }

				// if (changed)
				// {
				// 	double proxVec[5];

				// 	setEnableValves(0);
				// 	delay(waitTimePipePressurised);
				// 	b1.setPressure(pressureCmdVec[0]);
				// 	setEnableValves(1);
				// 	delay(waitTimeNextBellow);
				// 	proxVec[0] = b1.getProximity();

				// 	setEnableValves(0);
				// 	delay(waitTimePipePressurised);
				// 	b2.setPressure(pressureCmdVec[1]);
				// 	setEnableValves(2);
				// 	delay(waitTimeNextBellow);
				// 	proxVec[1] = b2.getProximity();

				// 	setEnableValves(0);
				// 	delay(waitTimePipePressurised);
				// 	b3.setPressure(pressureCmdVec[2]);
				// 	setEnableValves(3);
				// 	delay(waitTimeNextBellow);
				// 	proxVec[2] = b3.getProximity();

				// 	setEnableValves(0);
				// 	delay(waitTimePipePressurised);
				// 	b4.setPressure(pressureCmdVec[3]);
				// 	setEnableValves(4);
				// 	delay(waitTimeNextBellow);
				// 	proxVec[3] = b4.getProximity();

				// 	setEnableValves(0);
				// 	delay(waitTimePipePressurised);
				// 	b5.setPressure(pressureCmdVec[4]);
				// 	setEnableValves(5);
				// 	delay(waitTimeNextBellow);
				// 	proxVec[4] = b5.getProximity();

				// 	uint8_t rxHeader[2] = {0xAA, 0x55};
				// 	Serial1.write(rxHeader, sizeof(rxHeader));
				// 	Serial1.write((uint8_t *)proxVec, sizeof(proxVec));

				// 	Serial.print("prox: ");
				// 	for (uint8_t i = 0; i < 5; i++)
				// 	{
				// 		Serial.print(proxVec[i]);
				// 		if (i < 4)
				// 		{
				// 			Serial.print(", ");
				// 		}
				// 		prevPressureCmdVec[i] = pressureCmdVec[i];
				// 	}
				// 	Serial.println();

				// 	havePrevVec = true;
				// }
				double proxVec[5];

				// icc 1 bellow 1
				setEnableValves(0);
				delay(waitTimePipePressurised);
				b1.setPressure(pressureCmdVec[0]);
				setEnableValves(1);
				delay(waitTimeNextBellow);
				proxVec[0] = ((17.0 + b1.getProximityForICC()) * 4.5) - 78;

				// icc 2 bellow 2
				setEnableValves(0);
				delay(waitTimePipePressurised);
				b2.setPressure(pressureCmdVec[1]);
				setEnableValves(2);
				delay(waitTimeNextBellow);
				// proxVec[1] = 16 - b2.getProximityForICC();
				// proxVec[1] = 11 + b2.getProximityForICC();
				proxVec[1] = ((15.0 + b2.getProximityForICC()) * 3) - 78;

				// icc 3 bellow 3
				setEnableValves(0);
				delay(waitTimePipePressurised);
				b3.setPressure(pressureCmdVec[2]);
				setEnableValves(3);
				delay(waitTimeNextBellow);
				// proxVec[2] = 11.0 + b3.getProximityForICC();
				proxVec[2] = ((12.0 + b3.getProximityForICC()) * 5) - 78;

				// icc 4 bellow 4
				setEnableValves(0);
				delay(waitTimePipePressurised);
				b4.setPressure(pressureCmdVec[3]);
				setEnableValves(4);
				delay(waitTimeNextBellow);
				// proxVec[3] = 12.0 + b4.getProximityForICC();
				proxVec[3] = ((13.0 + b4.getProximityForICC()) * 5.5) - 78;

				// icc 5 bellow 5
				setEnableValves(0);
				delay(waitTimePipePressurised);
				b5.setPressure(pressureCmdVec[4]);
				setEnableValves(5);
				delay(waitTimeNextBellow);
				// proxVec[4] = 12.0 + b5.getProximityForICC(); // for ICC mapping, invert proximity so that 16mm is zero and smaller values are more negative
				proxVec[4] = ((14.0 + b5.getProximityForICC()) * 4.5) - 78;

				uint8_t rxHeader[2] = {0xAA, 0x55};
				Serial1.write(rxHeader, sizeof(rxHeader));
				Serial1.write((uint8_t *)proxVec, sizeof(proxVec));

				Serial.print("prox: ");
				for (uint8_t i = 0; i < 5; i++)
				{
					Serial.print(proxVec[i]);
					if (i < 4)
					{
						Serial.print(", ");
					}
				}
				Serial.println();
			}
		}

		setEnableValves(0); // close all enable valves
		break;
	}
}
