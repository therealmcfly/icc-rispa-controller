#include <Arduino.h>
#include <Wire.h>		// library for I2C control
#include <Bellow.h> // include header for bellow control
#include "icc.h"
#include "path.h"

#define TIME_STEP_MS_SINGLE 100
#define TIME_STEP_MS_1D 500
#define CONTRACTION_THRESHOLD -30.0f

// ICC LAYER CONFIGURATION
#define ICC_H_COUNT 5
#define ICC_V_COUNT 1
#define PACEMAKER_CELL_ROW 0
#define PACEMAKER_CELL_COL 4

Icc icc;
uint32_t step_interval_ms = TIME_STEP_MS_SINGLE;
uint32_t next_step_ms = 0;

// declare and initialise variables
#define waitTimeSymmetric 100			 // ms (symmetric mode)
#define waitTimePipePressurised 30 // ms
#define waitTimeNextBellow 50			 // ms
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

// ICC state variables initialisation
// IccConfig g_cfg;
// Icc icc;
// const uint32_t step_interval_ms = 10;
// uint32_t next_step_ms = 0;
// float icc_v = 0.0;

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
		switch (operationMode)
		{
		case (off):
			operationMode = single_icc;
			break;
		case (single_icc):
			operationMode = single_icc_new;
			break;
		case (single_icc_new):
			operationMode = icc_1d;
			break;
		case (icc_1d):
			operationMode = icc_1d_new;
			break;
		case (icc_1d_new):
			operationMode = off;
			break;
		default:
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

const char *modeToString(mode m)
{
	switch (m)
	{
	case off:
		return "off";
	case single_icc:
		return "single_icc";
	case single_icc_new:
		return "single_icc_new";
	case icc_1d:
		return "1d_icc";
	case icc_1d_new:
		return "1d_icc_new";
	case symmetric:
		return "symmetric";
	default:
		return "unknown";
	}
}

// Convert millivolts to kilopascals
int mv_to_kpa(float mv)
{
	const float vmin = -70.0;
	const float vmax = -23.5;
	const float pmin = -50.0;
	const float pmax = 55.0;

	float p = pmin + ((mv - vmin) / (vmax - vmin)) * (pmax - pmin);
	p = constrain(p, pmin, pmax); // Arduino's constrain() function

	return round(p);
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
		setAllOperationModes(symmetric);
		setAllPressures(0);
		break;

	case single_icc:
	{
		icc_init(&icc, true);

		step_interval_ms = TIME_STEP_MS_SINGLE;
		next_step_ms = 0;

		// Bellow initialisation for ICC mode
		setAllOperationModes(symmetric);
		setAllPressures(0);
		delay(100); // let pressure settle
		setEnableValves(0);
		delay(100);					// let pressure settle
		setEnableValves(6); // enable all bellows
		int pressureCmd = 0;
		int prevPressureCmd = 0;

		//
		next_step_ms = millis() + step_interval_ms;

		while (operationMode == single_icc)
		{
			if ((int32_t)(millis() - next_step_ms) >= 0)
			{
				(void)icc_update(&icc, step_interval_ms);

				next_step_ms += step_interval_ms;

				pressureCmd = mv_to_kpa(icc.v);

				if (prevPressureCmd != pressureCmd)
				{
					// b1.setPressure(pressureCmd);
					setAllPressures(pressureCmd);

					prevPressureCmd = pressureCmd;
				}

				// Send signal to PC
				uint8_t rxHeader[2] = {0xAA, 0x55};
				Serial1.write(rxHeader, sizeof(rxHeader));
				Serial1.write((uint8_t *)&icc.v, sizeof(icc.v));
				float prox = (float)((17.0 + b1.getProximityForICC()) * 4.5) - 78;
				Serial1.write((uint8_t *)&prox, sizeof(prox));
			}
		}
		break;
	}
	case single_icc_new:
	{
		icc_init(&icc, true);

		step_interval_ms = TIME_STEP_MS_SINGLE;
		next_step_ms = 0;

		// Bellow initialisation for ICC mode
		setAllOperationModes(symmetric);
		setAllPressures(0);
		delay(100); // let pressure settle
		setEnableValves(0);
		delay(100);					// let pressure settle
		setEnableValves(6); // enable all bellows
		int is_contracting = 0;
		int prev_is_contracting = 0;

		//
		next_step_ms = millis() + step_interval_ms;

		while (operationMode == single_icc_new)
		{
			if ((int32_t)(millis() - next_step_ms) >= 0)
			{
				(void)icc_update(&icc, step_interval_ms);

				next_step_ms += step_interval_ms;
				if (icc.v > CONTRACTION_THRESHOLD)
				{
					is_contracting = 1;
				}
				else
				{
					is_contracting = 0;
				}

				if (prev_is_contracting != is_contracting)
				{
					is_contracting ? setAllPressures(80) : setAllPressures(-50);
					prev_is_contracting = is_contracting;
				}

				// Send signal to PC
				uint8_t rxHeader[2] = {0xAA, 0x55};
				Serial1.write(rxHeader, sizeof(rxHeader));
				Serial1.write((uint8_t *)&icc.v, sizeof(icc.v));
				float prox = (float)((17.0 + b1.getProximityForICC()) * 4.5) - 78;
				Serial1.write((uint8_t *)&prox, sizeof(prox));
			}
		}
		break;
	}
	case icc_1d:
	{
		// ICC Variables
		Icc iccs[ICC_V_COUNT][ICC_H_COUNT];
		IccPath h_paths[ICC_V_COUNT][ICC_H_COUNT - 1]; // connects icc (i,j) to (i,j+1)
		float h_path_t1[ICC_V_COUNT][ICC_H_COUNT - 1]; // time from start of path activation to relay activation
		float h_path_t2[ICC_V_COUNT][ICC_H_COUNT - 1]; // time from start of path activation to other cell activation
#if (ICC_V_COUNT > 1)
		IccPath v_paths[ICC_V_COUNT - 1][ICC_H_COUNT]; // connects icc (i,j) to (i+1,j)
		float v_path_t1[ICC_V_COUNT - 1][ICC_H_COUNT]; // time from start of path activation to relay activation
		float v_path_t2[ICC_V_COUNT - 1][ICC_H_COUNT]; // time from start of path activation to other cell activation
#endif

		float proxLogVec[ICC_V_COUNT][ICC_H_COUNT];
		// Bellow
		Bellow *bellows[ICC_V_COUNT][ICC_H_COUNT];

		step_interval_ms = TIME_STEP_MS_1D;
		next_step_ms = 0;

		// initialise ICCs
		for (size_t i = 0; i < ICC_V_COUNT; i++)
		{
			for (size_t j = 0; j < ICC_H_COUNT; j++)
			{
				if (i == PACEMAKER_CELL_ROW && j == PACEMAKER_CELL_COL)
				{
					icc_init(&iccs[i][j], true);
				}
				else
				{
					icc_init(&iccs[i][j], false);
				}
			}
		}

		// initialise h_paths
		for (size_t i = 0; i < ICC_V_COUNT; i++)
		{
			for (size_t j = 0; j < ICC_H_COUNT - 1; j++)
			{
				icc_path_init(&h_paths[i][j], &h_path_t1[i][j], &h_path_t2[i][j]);
				h_paths[i][j].cells[0] = &iccs[i][j];
				h_paths[i][j].cells[1] = &iccs[i][j + 1];
			}
		}

		// initialise v_paths
#if (ICC_V_COUNT > 1)
		for (size_t i = 0; i < ICC_V_COUNT - 1; i++)
		{
			for (size_t j = 0; j < ICC_H_COUNT; j++)
			{
				icc_path_init(&v_paths[i][j], &v_path_t1[i][j], &v_path_t2[i][j]);
				v_paths[i][j].cells[0] = &iccs[i][j];
				v_paths[i][j].cells[1] = &iccs[i + 1][j];
			}
		}
#endif

		// initialize bellows
		bellows[0][0] = &b1;
		bellows[0][1] = &b2;
		bellows[0][2] = &b3;
		bellows[0][3] = &b4;
		bellows[0][4] = &b5;

		// Bellow initialisation for ICC mode
		setAllOperationModes(symmetric);
		setAllPressures(0);
		delay(1000); // let any late startup bytes arrive
		setEnableValves(0);

		next_step_ms = millis() + step_interval_ms;

		while (operationMode == icc_1d)
		{
			if ((int32_t)(millis() - next_step_ms) >= 0)
			{
				// Update ICCs
				for (size_t i = 0; i < ICC_V_COUNT; i++)
				{
					for (size_t j = 0; j < ICC_H_COUNT; j++)
					{
						(void)icc_update(&iccs[i][j], step_interval_ms);
						bellows[i][j]->setPressure(mv_to_kpa(iccs[i][j].v));
						delay(waitTimePipePressurised);
						setEnableValves(j + 1);
						delay(waitTimeNextBellow);
						setEnableValves(0);
						proxLogVec[i][j] = bellows[i][j]->getProximityForICC();
					}
				}

				// Update paths
				for (size_t i = 0; i < ICC_V_COUNT; i++)
				{
					for (size_t j = 0; j < ICC_H_COUNT - 1; j++)
					{
						icc_path_update(&h_paths[i][j], &h_path_t1[i][j], &h_path_t2[i][j], step_interval_ms);
					}
				}

				// Schedule next step
				next_step_ms += step_interval_ms;

				// icc 1 bellow 1
				proxLogVec[0][0] = ((17.0 + proxLogVec[0][0]) * 4.5) - 78;
				// icc 2 bellow 2
				proxLogVec[0][1] = ((15.0 + proxLogVec[0][1]) * 3) - 78;
				// icc 3 bellow 3
				proxLogVec[0][2] = ((12.0 + proxLogVec[0][2]) * 5) - 78;
				// icc 4 bellow 4
				proxLogVec[0][3] = ((13.0 + proxLogVec[0][3]) * 5.5) - 78;
				// icc 5 bellow 5
				proxLogVec[0][4] = ((14.0 + proxLogVec[0][4]) * 4.5) - 78;

				// Send ICC states over Serial1
				uint8_t rxHeader[2] = {0xAA, 0x55};
				Serial1.write(rxHeader, sizeof(rxHeader));
				// send ICC voltages as float
				for (size_t i = 0; i < ICC_V_COUNT; i++)
				{
					for (size_t j = 0; j < ICC_H_COUNT; j++)
					{
						Serial1.write((uint8_t *)&iccs[i][j].v, sizeof(float));
					}
				}
				for (size_t i = 0; i < ICC_V_COUNT; i++)
				{
					for (size_t j = 0; j < ICC_H_COUNT; j++)
					{
						Serial1.write((uint8_t *)&proxLogVec[i][j], sizeof(proxLogVec[i][j]));
					}
				}
			}
		}
		break;
	}

	case icc_1d_new:
	{
		// ICC Variables
		Icc iccs[ICC_V_COUNT][ICC_H_COUNT];
		IccPath h_paths[ICC_V_COUNT][ICC_H_COUNT - 1]; // connects icc (i,j) to (i,j+1)
		float h_path_t1[ICC_V_COUNT][ICC_H_COUNT - 1]; // time from start of path activation to relay activation
		float h_path_t2[ICC_V_COUNT][ICC_H_COUNT - 1]; // time from start of path activation to other cell activation
#if (ICC_V_COUNT > 1)
		IccPath v_paths[ICC_V_COUNT - 1][ICC_H_COUNT]; // connects icc (i,j) to (i+1,j)
		float v_path_t1[ICC_V_COUNT - 1][ICC_H_COUNT]; // time from start of path activation to relay activation
		float v_path_t2[ICC_V_COUNT - 1][ICC_H_COUNT]; // time from start of path activation to other cell activation
#endif
		// Bellow
		Bellow *bellows[ICC_V_COUNT][ICC_H_COUNT];
		float proxLogVec[ICC_V_COUNT][ICC_H_COUNT];

		step_interval_ms = TIME_STEP_MS_1D;
		next_step_ms = 0;

		// initialise ICCs
		for (size_t i = 0; i < ICC_V_COUNT; i++)
		{
			for (size_t j = 0; j < ICC_H_COUNT; j++)
			{
				if (i == PACEMAKER_CELL_ROW && j == PACEMAKER_CELL_COL)
				{
					icc_init(&iccs[i][j], true);
				}
				else
				{
					icc_init(&iccs[i][j], false);
				}
			}
		}

		// initialise h_paths
		for (size_t i = 0; i < ICC_V_COUNT; i++)
		{
			for (size_t j = 0; j < ICC_H_COUNT - 1; j++)
			{
				icc_path_init(&h_paths[i][j], &h_path_t1[i][j], &h_path_t2[i][j]);
				h_paths[i][j].cells[0] = &iccs[i][j];
				h_paths[i][j].cells[1] = &iccs[i][j + 1];
			}
		}

		// initialise v_paths
#if (ICC_V_COUNT > 1)
		for (size_t i = 0; i < ICC_V_COUNT - 1; i++)
		{
			for (size_t j = 0; j < ICC_H_COUNT; j++)
			{
				icc_path_init(&v_paths[i][j], &v_path_t1[i][j], &v_path_t2[i][j]);
				v_paths[i][j].cells[0] = &iccs[i][j];
				v_paths[i][j].cells[1] = &iccs[i + 1][j];
			}
		}
#endif

		// initialize bellows
		bellows[0][0] = &b1;
		bellows[0][1] = &b2;
		bellows[0][2] = &b3;
		bellows[0][3] = &b4;
		bellows[0][4] = &b5;

		// Bellow initialisation for ICC mode
		setAllOperationModes(symmetric);
		setAllPressures(0);
		delay(1000); // let any late startup bytes arrive
		setEnableValves(0);

		next_step_ms = millis() + step_interval_ms;

		while (operationMode == icc_1d_new)
		{
			if ((int32_t)(millis() - next_step_ms) >= 0)
			{
				// Update ICCs
				for (size_t i = 0; i < ICC_V_COUNT; i++)
				{
					for (size_t j = 0; j < ICC_H_COUNT; j++)
					{
						(void)icc_update(&iccs[i][j], step_interval_ms);
						bellows[i][j]->setPressure(iccs[i][j].v > CONTRACTION_THRESHOLD ? 80 : -50);
						delay(waitTimePipePressurised);
						setEnableValves(j + 1);
						delay(waitTimeNextBellow);
						setEnableValves(0);
						proxLogVec[i][j] = bellows[i][j]->getProximityForICC();
					}
				}

				// Update paths
				for (size_t i = 0; i < ICC_V_COUNT; i++)
				{
					for (size_t j = 0; j < ICC_H_COUNT - 1; j++)
					{
						icc_path_update(&h_paths[i][j], &h_path_t1[i][j], &h_path_t2[i][j], step_interval_ms);
					}
				}

				// Schedule next step
				next_step_ms += step_interval_ms;

				// icc 1 bellow 1
				proxLogVec[0][0] = ((17.0 + proxLogVec[0][0]) * 4.5) - 78;
				// icc 2 bellow 2
				proxLogVec[0][1] = ((15.0 + proxLogVec[0][1]) * 3) - 78;
				// icc 3 bellow 3
				proxLogVec[0][2] = ((12.0 + proxLogVec[0][2]) * 5) - 78;
				// icc 4 bellow 4
				proxLogVec[0][3] = ((13.0 + proxLogVec[0][3]) * 5.5) - 78;
				// icc 5 bellow 5
				proxLogVec[0][4] = ((14.0 + proxLogVec[0][4]) * 4.5) - 78;

				// Send ICC states over Serial1
				uint8_t rxHeader[2] = {0xAA, 0x55};
				Serial1.write(rxHeader, sizeof(rxHeader));
				// send ICC voltages as float
				for (size_t i = 0; i < ICC_V_COUNT; i++)
				{
					for (size_t j = 0; j < ICC_H_COUNT; j++)
					{
						Serial1.write((uint8_t *)&iccs[i][j].v, sizeof(iccs[i][j].v));
					}
				}
				for (size_t i = 0; i < ICC_V_COUNT; i++)
				{
					for (size_t j = 0; j < ICC_H_COUNT; j++)
					{
						Serial1.write((uint8_t *)&proxLogVec[i][j], sizeof(proxLogVec[i][j]));
					}
				}
			}
		}
		break;
	}

	default:
		setAllOperationModes(symmetric);
		setAllPressures(0);
		break;
	}
}
