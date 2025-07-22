#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <TinyGPSPlus.h>
#include <Flysky.h>
#include <L298N.h>

// ----------------- STM32 ----------------- //
HardwareSerial Serial1(USART1);

uint32_t loop_timer;
uint8_t loop_counter;
// ----------------- STM32 ----------------- //

// ----------------- NRF24 ----------------- //
struct transmitter
{
	uint8_t manualMode;
	uint8_t no_of_sats;
	int16_t tagetheading;
	int16_t heading;
	int16_t distance;
	float latitude;
	float longitude;
	float targetlatitude;
	float targetlongitude;
} t_data;

struct receiver
{
	uint8_t command;
} r_data;

RF24 radio(PB0, PA4);

const byte address[6] = "00001";

bool transmitData();
// ----------------- NRF24 ----------------- //

// ----------------- Compass ----------------- //
#define MAGNETIC_DECLINATION 0.4
#define HEADING_TOLERANCE 5

QMC5883LCompass compass;

bool calibrationMode = false;

void getCurrentHeading();
void CalibrateCompass();
// ----------------- Compass ----------------- //

// ----------------- GNSS ----------------- //
struct GNSSData
{
	float latitude;
	float longitude;
	int16_t heading;
} GNSSTarget, GNSSCurrent, GNSSHome;

TinyGPSPlus GNSS;

volatile bool dataReady = false;

void onSerialReceive();
// ----------------- GNSS ----------------- //

// ----------------- WayPoint Following ----------------- //
#define WAYPOINT_DIST_TOLERANE 1
#define NUMBER_WAYPOINTS 20

struct WayPointInfo
{
	int16_t headingerror;
	int16_t distanceToTarget;
	int16_t originalDistanceToTarget;
	int8_t waypointNumber;
	int8_t waypointListNumber;
} WayPoint;

struct waypointStruct
{
	float latitude;
	float longitude;
} waypointList[NUMBER_WAYPOINTS];

void courseToWaypoint();
void calcDesiredTurn();
int headingerror();
void distanceToWaypoint();
void checkAndMoveToNextWaypoint();
// ----------------- WayPoint Following ----------------- //

// ----------------- Motor ----------------- //
uint8_t CH4_state;
uint8_t CH5_state;
uint8_t CH6_state;

#define PPM_PIN PB1
#define NUM_CHANNELS 6

FLYSKY FSReceiver(PPM_PIN, NUM_CHANNELS);

uint8_t motorCommand;

#define LEFT_IN1 PB14
#define LEFT_IN2 PB15
#define LEFT_EN PA1
#define RIGHT_IN1 PB12
#define RIGHT_IN2 PB13
#define RIGHT_EN PA0

L298N leftMotor(LEFT_EN, LEFT_IN1, LEFT_IN2);
L298N rightMotor(RIGHT_EN, RIGHT_IN1, RIGHT_IN2);

int16_t rightMotorSpeed;
int16_t leftMotorSpeed;

void MotorRun();
void MotorRun(int16_t leftMotorSpeed, int16_t rightMotorSpeed);
void MotorStop();
// ----------------- Motor ----------------- //

void setup()
{
	// ----------------- STM32 ----------------- //
	Serial1.begin(115200);
	// ----------------- STM32 ----------------- //

	// ----------------- NRF24 ----------------- //
	Serial1.println(radio.begin() ? "\nNRF24 Initialized" : "\nNRF24 Initialization failed");

	radio.setDataRate(RF24_1MBPS);
	radio.setPALevel(RF24_PA_MAX);
	radio.setChannel(124);
	radio.openWritingPipe(address);
	radio.openReadingPipe(1, address);
	radio.startListening();
	// ----------------- NRF24 ----------------- //

	// ----------------- Compass ----------------- //
	compass.init();
	do
	{
		getCurrentHeading();
		if (!GNSSCurrent.heading)
		{
			Serial1.println("compass not Initialized");
		}
		delay(50);
	} while (!GNSSCurrent.heading);
	compass.setMagneticDeclination(0, 24);
	Serial1.println("Compass Initialized");
	// ----------------- Compass ----------------- //

	// ----------------- Motor ----------------- //
	FSReceiver.attachInterrupt();

	MotorStop();
	// ----------------- Motor ----------------- //

	// ----------------- GNSS ----------------- //
	t_data.manualMode = true;
	WayPoint.waypointNumber = -1;
	WayPoint.waypointListNumber = -1;

	Serial.begin(115200);
	attachInterrupt(digitalPinToInterrupt(PA3), onSerialReceive, FALLING);

	GNSSHome.latitude = GNSS.location.lat();
	GNSSHome.longitude = GNSS.location.lng();
	GNSSHome.heading = compass.getAzimuth();

	Serial1.println("GNSS Fix obtained");
	// ----------------- GNSS ----------------- //

	// ----------------- STM32 ----------------- //
	loop_timer = millis();
	// ----------------- STM32 ----------------- //
}

void loop()
{
	if (loop_timer + 50 < millis())
	{
		loop_timer = millis();
		loop_counter++;

		if (radio.available())
		{
			radio.read(&r_data, sizeof(r_data));
			Serial1.println("Command received");

			if (r_data.command >= 1 && r_data.command <= 5)
			{
				t_data.manualMode = true;
			}

			if (r_data.command == 'c')
			{
				calibrationMode = true;
				Serial1.println("Calibration mode activated");
			}
			else if (r_data.command == 'm')
			{
				t_data.manualMode = true;
				Serial1.println("Manual mode activated");
			}
			else if (r_data.command == 'a')
			{
				t_data.manualMode = false;
				if (WayPoint.waypointListNumber != -1)
				{
					if (WayPoint.waypointNumber == -1)
						WayPoint.waypointNumber = 0;
					GNSSTarget.latitude = waypointList[WayPoint.waypointNumber].latitude;
					GNSSTarget.longitude = waypointList[WayPoint.waypointNumber].longitude;
				}
			}
			else if (r_data.command == 'w')
			{
				while (!radio.available())
					;
				Serial1.print("Waypoint received: ");
				if (WayPoint.waypointListNumber < NUMBER_WAYPOINTS - 1)
				{
					WayPoint.waypointListNumber++;
					radio.read(&waypointList[WayPoint.waypointListNumber], sizeof(waypointList[WayPoint.waypointListNumber]));
				}
				else
				{
					radio.flush_rx();
					Serial1.println("Waypoint list full");
				}
				Serial1.println(WayPoint.waypointListNumber);
			}
			else if (r_data.command == 6)
			{
				WayPoint.waypointNumber = -1;
				WayPoint.waypointListNumber = -1;
				t_data.manualMode = true;
				GNSSTarget.latitude = 0;
				GNSSTarget.longitude = 0;

				MotorStop();
			}
		}

		if (calibrationMode)
		{
			CalibrateCompass();
			calibrationMode = false;
		}

		getCurrentHeading();

		if (FSReceiver.getPpmValue(4) > 1800 && CH4_state != 1 && calibrationMode == false)
		{
			CH4_state = 1;
			calibrationMode = true;
		}
		else if (FSReceiver.getPpmValue(4) < 1700 && CH4_state != 0)
		{
			CH4_state = 0;
		}

		if (FSReceiver.getPpmValue(5) < 1200 && CH5_state != 0)
		{
			CH5_state = 0;
			t_data.manualMode = true;
		}
		else if (FSReceiver.getPpmValue(5) > 1800 && CH5_state != 1)
		{
			CH5_state = 1;
			t_data.manualMode = false;

			if (WayPoint.waypointListNumber != -1)
			{
				if (WayPoint.waypointNumber == -1)
					WayPoint.waypointNumber = 0;
				GNSSTarget.latitude = waypointList[WayPoint.waypointNumber].latitude;
				GNSSTarget.longitude = waypointList[WayPoint.waypointNumber].longitude;
			}
			else
				t_data.manualMode = true;
		}

		if (FSReceiver.getPpmValue(6) < 1200 && CH6_state != 0)
		{
			CH6_state = 0;
		}
		else if (FSReceiver.getPpmValue(6) > 1800 && CH6_state != 2)
		{
			CH6_state = 2;

			WayPoint.waypointNumber = -1;
			WayPoint.waypointListNumber = -1;
			t_data.manualMode = true;
			GNSSTarget.latitude = 0;
			GNSSTarget.longitude = 0;

			MotorStop();
		}

		if (loop_counter == 20)
		{
			GNSSCurrent.latitude = GNSS.location.lat();
			GNSSCurrent.longitude = GNSS.location.lng();

			transmitData();

			loop_counter = 0;
		}

		if (!t_data.manualMode)
		{
			courseToWaypoint();
			calcDesiredTurn();
			distanceToWaypoint();
			checkAndMoveToNextWaypoint();
		}
		else
		{
			leftMotorSpeed = rightMotorSpeed = map(FSReceiver.getPpmValue(2), 1000, 2000, -255, 255);
			if (FSReceiver.getPpmValue(1) <= 1500)
			{
				int16_t speed = map(FSReceiver.getPpmValue(1), 1000, 1500, -255, 0);
				if (leftMotorSpeed >= 0)
				{
					leftMotorSpeed = leftMotorSpeed + speed;
					rightMotorSpeed = max(rightMotorSpeed, (int16_t)(-1 * speed));
				}
				else
				{
					leftMotorSpeed = min(leftMotorSpeed, speed);
					rightMotorSpeed = rightMotorSpeed - speed;
				}
			}
			else
			{
				int16_t speed = map(FSReceiver.getPpmValue(1), 1500, 2000, 0, -255);
				if (leftMotorSpeed >= 0)
				{
					leftMotorSpeed = max(leftMotorSpeed, (int16_t)(-1 * speed));
					rightMotorSpeed = rightMotorSpeed + speed;
				}
				else
				{
					leftMotorSpeed = leftMotorSpeed - speed;
					rightMotorSpeed = min(rightMotorSpeed, speed);
				}
			}
			leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
			rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
			MotorRun();
		}
	}
}

// ----------------- Compass ----------------- //
void getCurrentHeading()
{
	compass.read();
	GNSSCurrent.heading = compass.getAzimuth();
}

void CalibrateCompass()
{
	MotorRun(255, -255);

	compass.calibrate();

	MotorStop();
}
// ----------------- Compass ----------------- //

// ----------------- NRF24 ----------------- //
bool transmitData()
{
	bool sent = true;

	radio.stopListening();

	t_data.distance = WayPoint.distanceToTarget;
	t_data.latitude = GNSSCurrent.latitude;
	t_data.longitude = GNSSCurrent.longitude;
	t_data.heading = GNSSCurrent.heading;
	t_data.tagetheading = GNSSTarget.heading;
	t_data.targetlatitude = GNSSTarget.latitude;
	t_data.targetlongitude = GNSSTarget.longitude;
	t_data.no_of_sats = GNSS.satellites.value();
	
	Serial1.print("Left: ");
	Serial1.print(leftMotorSpeed);
	Serial1.print("  Right: ");
	Serial1.println(rightMotorSpeed);

	// Serial1.print("Mode: ");
	// Serial1.println(t_data.manualMode);
	// Serial1.print("Distance: ");
	// Serial1.println(t_data.distance);
	// Serial1.print("Latitude: ");
	// Serial1.println(t_data.latitude, 6);
	// Serial1.print("Longitude: ");
	// Serial1.println(t_data.longitude, 6);
	// Serial1.print("Heading: ");
	// Serial1.println(t_data.heading);
	// Serial1.print("Target Heading: ");
	// Serial1.println(t_data.tagetheading);

	sent *= radio.write(&t_data, sizeof(t_data));
	radio.startListening();

	return sent;
}
// ----------------- NRF24 ----------------- //

// ----------------- Motor ----------------- //
void MotorRun()
{
	leftMotor.run(leftMotorSpeed);
	rightMotor.run(rightMotorSpeed);
}

void MotorRun(int16_t leftSpeed, int16_t rightSpeed)
{
	leftMotorSpeed = leftSpeed;
	rightMotorSpeed = rightSpeed;

	leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
	rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

	leftMotor.run(leftMotorSpeed);
	rightMotor.run(rightMotorSpeed);
}

void MotorStop()
{
	leftMotorSpeed = 0;
	rightMotorSpeed = 0;

	leftMotor.run(leftMotorSpeed);
	rightMotor.run(rightMotorSpeed);
}
// ----------------- Motor ----------------- //

// ----------------- WayPoint Following ----------------- //
void courseToWaypoint()
{
	float dlon = radians(GNSSTarget.longitude - GNSSCurrent.longitude);
	float cLat = radians(GNSSCurrent.latitude);
	float tLat = radians(GNSSTarget.latitude);
	float a1 = sin(dlon) * cos(tLat);
	float a2 = sin(cLat) * cos(tLat) * cos(dlon);
	a2 = cos(cLat) * sin(tLat) - a2;
	a2 = atan2(a1, a2);
	// if (a2 < 0.0)
	// {
	//     a2 += TWO_PI;
	// }
	GNSSTarget.heading = degrees(a2);
}

void calcDesiredTurn()
{
	int headingError = headingerror();

	if (abs(headingError) <= HEADING_TOLERANCE)
	{
		MotorRun(255, 255);
		// MotorStop();
	}
	else if (headingError < 0)
	{
		do
		{
			MotorRun(-255, 255);
			getCurrentHeading();
			headingError = headingerror();
		} while (!(abs(headingError) <= HEADING_TOLERANCE - 2));
	}
	else if (headingError > 0)
	{
		do
		{
			MotorRun(255, -255);
			getCurrentHeading();
			headingError = headingerror();
		} while (!(abs(headingError) <= HEADING_TOLERANCE - 2));
	}
	else
	{
		MotorRun(255, 255);
		// MotorStop();
	}
}

int headingerror()
{
	int headingError = GNSSTarget.heading - GNSSCurrent.heading;

	if (headingError < -180)
		headingError += 360;
	if (headingError > 180)
		headingError -= 360;
	return headingError;
}

void distanceToWaypoint()
{
	float delta = radians(GNSSCurrent.longitude - GNSSTarget.longitude);
	float sdlong = sin(delta);
	float cdlong = cos(delta);
	float lat1 = radians(GNSSCurrent.latitude);
	float lat2 = radians(GNSSTarget.latitude);
	float slat1 = sin(lat1);
	float clat1 = cos(lat1);
	float slat2 = sin(lat2);
	float clat2 = cos(lat2);
	delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
	delta = sq(delta);
	delta += sq(clat2 * sdlong);
	delta = sqrt(delta);
	float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
	delta = atan2(delta, denom);
	WayPoint.distanceToTarget = delta * 6372795;

	if (WayPoint.distanceToTarget <= WAYPOINT_DIST_TOLERANE)
		WayPoint.waypointNumber++;
}

void checkAndMoveToNextWaypoint()
{
	if (WayPoint.waypointNumber > WayPoint.waypointListNumber)
	{
		t_data.manualMode = true;
		MotorStop();
		WayPoint.waypointNumber = -1;
		WayPoint.waypointListNumber = -1;
	}
	else
	{
		GNSSTarget.latitude = waypointList[WayPoint.waypointNumber].latitude;
		GNSSTarget.longitude = waypointList[WayPoint.waypointNumber].longitude;
	}
}
// ----------------- WayPoint Following ----------------- //

// ----------------- GNSS ----------------- //
void onSerialReceive()
{
	while (Serial.available())
		if (GNSS.encode(Serial.read()))
			dataReady = true;
}
// ----------------- GNSS ----------------- //