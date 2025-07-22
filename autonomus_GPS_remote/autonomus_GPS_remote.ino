#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <TinyGPSPlus.h>

uint32_t loopTimer;
uint8_t loopCounter;

TinyGPSPlus GNSS;

volatile bool dataReady = false;

void onSerialReceive()
{
    while (Serial2.available())
        if (GNSS.encode(Serial2.read()))
            dataReady = true;
}

struct GNSSLocation
{
	float latitude;
	float longitude;
} waypoints[30], Current;

#define START_RECORD T4
#define PAUSE_RECORD T6
#define RESET_RECORD T5
#define START_TRACING T7
#define PAUSE_TRACING T9
#define RESET_TRACING T8

#define TOUCH_THRESHOLD 20

// #define DISTANCE_TOLERANCE 5
long DISTANCE_TOLERANCE = 5;

uint8_t recordState = 0;
uint8_t tracingStatus = 0;
int8_t waypointCounter = -1;
int8_t startIdx = 0; // Start index of the circular buffer
int8_t endIdx = 0;	 // End index of the circular buffer

double distanceBetween(double lat1, double long1, double lat2, double long2);

void handleRecordState();
void handleTracingState();
void recordWaypoint();
void startTracing();
void pauseTracing();
void resetTracing();

// void test_recordWaypoint();

struct Transmitter
{
	uint8_t command;
} tData;

struct Receiver
{
	uint8_t manualMode;
	int16_t tagetheading;
	int16_t heading;
	int16_t distance;
	float latitude;
	float longitude;
	float targetlatitude;
	float targetlongitude;
} rData;

RF24 radio(4, 5);

const byte address[6] = "00001";

bool transmitData(void *buff, uint8_t length);

void setup()
{
	Serial.begin(115200);
	Serial2.begin(115200);
	Serial2.onReceive(onSerialReceive, false);

	Serial.println("GNSS Module Ready!");

	Serial.println(radio.begin() ? "\nNRF24 Initialized" : "\nNRF24 Initialization failed");

	radio.setDataRate(RF24_1MBPS);
	radio.setPALevel(RF24_PA_MAX);
	radio.setChannel(124);
	radio.openWritingPipe(address);
	radio.openReadingPipe(1, address);

	delay(5);
	radio.startListening();

	pinMode(START_RECORD, INPUT_PULLUP);
	pinMode(PAUSE_RECORD, INPUT_PULLUP);
	pinMode(RESET_RECORD, INPUT_PULLUP);
	pinMode(START_TRACING, INPUT_PULLUP);
	pinMode(PAUSE_TRACING, INPUT_PULLUP);
	pinMode(RESET_TRACING, INPUT_PULLUP);

	loopTimer = millis();

	// tData.command = 'y';
}

void loop()
{
	handleRecordState();
	handleTracingState();

	if (loopCounter == 50)
	{
		Current.latitude = GNSS.location.lat();
		Current.longitude = GNSS.location.lng();

		loopCounter = 0;

		if (recordState == 1)
		{
			recordWaypoint();
			// test_recordWaypoint();
		}

		Serial.print("Distance tolerence: ");
		Serial.print(DISTANCE_TOLERANCE);
		Serial.print("   Record State: ");
		Serial.print(recordState);
		Serial.print("   Tracing status: ");
		Serial.print(tracingStatus);
		Serial.print("   Waypoint counter: ");
		Serial.println(waypointCounter);

		// transmitData(&tData, sizeof(Transmitter));
	}

	if (Serial.available())
	{
		tData.command = Serial.read();

		if (tData.command == 'D')
		{
			String input = Serial.readStringUntil('\n');
			input.trim();
			Serial.print(input);
			DISTANCE_TOLERANCE = input.toInt();
		}

		if (transmitData(&tData, sizeof(Transmitter)))
		{
			Serial.println("Sent Data");
		}
		else
		{
			Serial.println("Failed to send Data");
		}
	}

	if (radio.available())
	{
		radio.read(&rData, sizeof(Receiver));

		Serial.print("\n\n");
		Serial.print("Mode: ");
		Serial.println(rData.manualMode);
		Serial.print("Target Heading: ");
		Serial.println(rData.tagetheading);
		Serial.print("Current Heading: ");
		Serial.println(rData.heading);
		Serial.print("Current Distance: ");
		Serial.println(rData.distance);
		Serial.print("Current Latitude: ");
		Serial.println(rData.latitude, 6);
		Serial.print("Current Longitude: ");
		Serial.println(rData.longitude, 6);
		Serial.print("Target Latitude: ");
		Serial.println(rData.targetlatitude, 6);
		Serial.print("Target Longitude: ");
		Serial.println(rData.targetlongitude, 6);
		Serial.print("Remote Latitude: ");
		Serial.println(Current.latitude, 6);
		Serial.print("Remote Longitude: ");
		Serial.println(Current.longitude, 6);
	}

	while (millis() - loopTimer < 20)
		;
	loopTimer = millis();
	loopCounter++;
}

void handleRecordState()
{
  if (digitalRead(START_RECORD) == LOW && recordState != 1)
  {
    Serial.println("Start record");
    recordState = 1;
  }
  else if (digitalRead(PAUSE_RECORD) == LOW && recordState != 0)
  {
    Serial.println("Pause record");
    recordState = 0;
  }
  else if (digitalRead(RESET_RECORD) == LOW && waypointCounter != -1)
  {
    Serial.println("Reset record");
    waypointCounter = -1;
    startIdx = 0;
    endIdx = 0;
  }
}

void handleTracingState()
{
  if (digitalRead(START_TRACING) == LOW && tracingStatus != 1 && recordState == 0 && waypointCounter != -1)
  {
    startTracing();
  }
  else if (digitalRead(PAUSE_TRACING) == LOW && tracingStatus != 0 && recordState == 0)
  {
    pauseTracing();
  }
  else if (digitalRead(RESET_TRACING) == LOW && recordState == 0)
  {
    resetTracing();
  }
}

// void handleRecordState()
// {
// 	if (touchRead(START_RECORD) < TOUCH_THRESHOLD && recordState != 1)
// 	{
// 		Serial.println("Start record");
// 		recordState = 1;
// 	}
// 	else if (touchRead(PAUSE_RECORD) < TOUCH_THRESHOLD && recordState != 0)
// 	{
// 		Serial.println("Pause record");
// 		recordState = 0;
// 	}
// 	else if (touchRead(RESET_RECORD) < TOUCH_THRESHOLD && waypointCounter != -1)
// 	{
// 		Serial.println("Reset record");
// 		waypointCounter = -1;
// 		startIdx = 0;
// 		endIdx = 0;
// 	}
// }

// void handleTracingState()
// {
// 	if (touchRead(START_TRACING) < TOUCH_THRESHOLD && tracingStatus != 1 && recordState == 0 && waypointCounter != -1)
// 	{
// 		startTracing();
// 	}
// 	else if (touchRead(PAUSE_TRACING) < TOUCH_THRESHOLD && tracingStatus != 0 && recordState == 0)
// 	{
// 		pauseTracing();
// 	}
// 	else if (touchRead(RESET_TRACING) < TOUCH_THRESHOLD && recordState == 0)
// 	{
// 		resetTracing();
// 	}
// }

void recordWaypoint()
{
	if (waypointCounter == -1 && Current.latitude != 0.0 && Current.longitude != 0.0)
	{
		waypointCounter++;
		waypoints[endIdx].latitude = Current.latitude;
		waypoints[endIdx].longitude = Current.longitude;
	}
	else if (distanceBetween(Current.latitude, Current.longitude, waypoints[endIdx].latitude, waypoints[endIdx].longitude) > DISTANCE_TOLERANCE)
	{
		waypointCounter++;
		endIdx = (endIdx + 1) % 30;
		if (endIdx == startIdx)
		{
			startIdx = (startIdx + 1) % 30;
		}
		waypoints[endIdx].latitude = Current.latitude;
		waypoints[endIdx].longitude = Current.longitude;
	}
}

// void test_recordWaypoint()
// {
//   while (Serial.available())
//   {
//     tData.command = (uint8_t)Serial.read();

//     if (tData.command == 'w')
//     {
//       delay(5);
//       String input = Serial.readStringUntil(',');
//       input.trim();
//       Current.latitude = input.toFloat();
//       input = Serial.readStringUntil(',');
//       input.trim();
//       Current.longitude = input.toFloat();

//       Serial.print("Waypoint: ");
//       Serial.print(Current.latitude, 6);
//       Serial.print(" ");
//       Serial.println(Current.longitude, 6);
//     }
//   }

//   if (waypointCounter == -1)
//   {
//     waypointCounter++;
//     waypoints[endIdx].latitude = Current.latitude;
//     waypoints[endIdx].longitude = Current.longitude;
//   }
//   else if (distanceBetween(Current.latitude, Current.longitude, waypoints[endIdx].latitude, waypoints[endIdx].longitude) > DISTANCE_TOLERANCE)
//   {
//     waypointCounter++;
//     endIdx = (endIdx + 1) % 30;
//     if (endIdx == startIdx)
//     {
//       startIdx = (startIdx + 1) % 30;
//     }
//     waypoints[endIdx].latitude = Current.latitude;
//     waypoints[endIdx].longitude = Current.longitude;
//   }
// }

void startTracing()
{
	Serial.println("Start tracking");
	tracingStatus = 1;
	tData.command = 'w';
	int8_t idx = startIdx;
	do
	{
		if (transmitData(&tData, sizeof(tData)))
		{
			Serial.println("Sent Data");
		}
		else
		{
			Serial.println("Failed to send Data");
		}
		delay(5);
		if (transmitData(&waypoints[idx], sizeof(GNSSLocation)))
		{
			Serial.println("Sent waypoint");
		}
		else
		{
			Serial.println("Failed to send waypoint");
		}
		delay(5);
		idx = (idx + 1) % 30;
		delay(20);
	} while (idx != (endIdx + 1) % 30);

	tData.command = 'a';
	if (transmitData(&tData, sizeof(tData)))
	{
		Serial.println("Sent Data");
	}
	else
	{
		Serial.println("Failed to send Data");
	}

	waypointCounter = -1;
	startIdx = 0;
	endIdx = 0;
}

void pauseTracing()
{
	Serial.println("Pause tracking");
	tracingStatus = 0;
	tData.command = 'm';
	if (transmitData(&tData, sizeof(tData)))
	{
		Serial.println("Sent Data");
	}
	else
	{
		Serial.println("Failed to send Data");
	}
}

void resetTracing()
{
	Serial.println("Reset tracking");
	tracingStatus = 0;
	tData.command = 6;
	if (transmitData(&tData, sizeof(tData)))
	{
		Serial.println("Sent Data");
	}
	else
	{
		Serial.println("Failed to send Data");
	}
}

double distanceBetween(double lat1, double long1, double lat2, double long2)
{
	// returns distance in meters between two positions, both specified
	// as signed decimal-degrees latitude and longitude. Uses great-circle
	// distance computation for hypothetical sphere of radius 6371009 meters.
	// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
	// Courtesy of Maarten Lamers
	double delta = radians(long1 - long2);
	double sdlong = sin(delta);
	double cdlong = cos(delta);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	double slat1 = sin(lat1);
	double clat1 = cos(lat1);
	double slat2 = sin(lat2);
	double clat2 = cos(lat2);
	delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
	delta = sq(delta);
	delta += sq(clat2 * sdlong);
	delta = sqrt(delta);
	double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
	delta = atan2(delta, denom);
	return delta * 6371009;
}

bool transmitData(void *buff, uint8_t length)
{
	bool sent;
	radio.stopListening();
	sent = radio.write(buff, length);
	radio.startListening();
	return sent;
}