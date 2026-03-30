#include <Arduino.h>
#include <Adafruit_VCNL4040.h>

// create enum for operation modes for better code readability
enum mode
{
	off = 0,
	symmetric = 1,
	asymmetric = 2,
	icc = 3,
	icc_1d = 4
};

// declaration of new class Bellow
class Bellow
{
private:
// private member variables
#define minAirPressure -50 // kPa
#define maxAirPressure 80	 // kPa
#define pressureValvePin 5
#define pressureValveSensorPin A15
#define vacuumValvePin 2
#define vacuumValveSensorPin A14
#define TCAADDR 0x70

	int proximitySensorPort;
	int forceSensorPin;
	int enableValvePin;
	int switchValvePin;
	double proximityParameterA;
	double proximityParameterB;
	double fsrParameterA;
	double fsrParameterB;
	int pressure;

	mode operationMode;
	Adafruit_VCNL4040 proximitySensor = Adafruit_VCNL4040();

	// declaration of private member function
	void tcaPortSelect();

public:
	// Constructor
	Bellow(int proximitySensorPort, int forceSensorPin, int switchValvePin, int enableValvePin, double proximityParameterA, double proximityParameterB, double fsrParameterA, double fsrParameterB)
	{
		this->proximitySensorPort = proximitySensorPort;
		this->forceSensorPin = forceSensorPin;
		this->enableValvePin = enableValvePin;
		this->switchValvePin = switchValvePin;
		this->proximityParameterA = proximityParameterA;
		this->proximityParameterB = proximityParameterB;
		this->fsrParameterA = fsrParameterA;
		this->fsrParameterB = fsrParameterB;
		pinMode(forceSensorPin, INPUT);
		pinMode(pressureValveSensorPin, INPUT);
		pinMode(vacuumValveSensorPin, INPUT);
		pinMode(enableValvePin, OUTPUT);
		pinMode(switchValvePin, OUTPUT);
	};

	// declaration of public member functions
	void setupProximitySensor();

	double getForce();
	double getProximity();
	double getPressure();

	double getProximityForICC();

	void setOperationMode(mode operationMode);
	void setPressure(int pressure);
	void setEnableValve(bool value);
	void setSwitchValve(bool value);
};