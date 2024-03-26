#include <Servo.h>
#include <AFMotor.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>

#define Echo A0
#define Trig A1
#define motor 10
#define Speed 170
#define spoint 103
#define SAFE_DISTANCE 20
#define THRESHOLD 0.001

char value;
long distance;
long Left;
long Right;
long L = 0;
long R = 0;
long L1 = 0;
long R1 = 0;

const double dest_lat = 23.726;
const double dest_lng = 90.0904;

struct
{
	bool dest_reached;
	unsigned int no_of_turns_taken;
} params;

Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

static const int RXPin = 2, TXPin = 8; // Here we make pin 4 as RX of arduino & pin 3 as TX of arduino
static const long BLUETOOTH_BAUD = 9600;
static const long GPS_BAUD = 9600;
int cnt = 0;
double lat_sum = 0.0;
double lon_sum = 0.0;

TinyGPSPlus gps;

SoftwareSerial bs(RXPin, TXPin);

void setup()
{
	params.dest_reached = false;
	params.no_of_turns_taken = 0;
	Serial.begin(GPS_BAUD);
	pinMode(Trig, OUTPUT);
	pinMode(Echo, INPUT);
	// servo.attach(motor);
	M1.setSpeed(Speed);
	M2.setSpeed(Speed);
	M3.setSpeed(Speed);
	M4.setSpeed(Speed);
	bs.begin(BLUETOOTH_BAUD);
	delay(200);
}
void loop()
{
	// Obstacle();
	// Bluetoothcontrol();
	// voicecontrol();
	gpsControl();
}

void Bluetoothcontrol()
{
	if (bs.available() > 0)
	{
		value = bs.read();
		bs.println(value);
	}
	if (value == 'F')
	{
		forward();
	}
	else if (value == 'B')
	{
		backward();
	}
	else if (value == 'L')
	{
		left();
	}
	else if (value == 'R')
	{
		right();
	}
	else if (value == 'S')
	{
		Stop();
	}
}

void Obstacle()
{
	bs.print(F("Distance : "));
	distance = ultrasonic();
	bs.println(distance);
	if (distance <= SAFE_DISTANCE)
	{
		Stop();
		backward();
		delay(100);
		Stop();
		L = leftsee();
		servo.write(spoint);
		delay(800);
		R = rightsee();
		servo.write(spoint);
		if (L < R && R >= SAFE_DISTANCE)
		{
			right();
			delay(900);
			Stop();
			delay(200);
		}
		else if (L > R && L >= SAFE_DISTANCE)
		{
			left();
			delay(900);
			Stop();
			delay(200);
		}
		else
		{
			backward();
			delay(600);
			Stop();
			delay(200);
		}
	}
	else
	{
		forward();
	}
}

void voicecontrol()
{
	if (bs.available() > 0)
	{
		value = bs.read();
		bs.println(value);
		if (value == '^')
		{
			forward();
		}
		else if (value == '-')
		{
			backward();
		}
		else if (value == '<')
		{
			L = leftsee();
			servo.write(spoint);
			if (L >= 10)
			{
				left();
				delay(500);
				Stop();
			}
			else if (L < 10)
			{
				Stop();
			}
		}
		else if (value == '>')
		{
			R = rightsee();
			servo.write(spoint);
			if (R >= 10)
			{
				right();
				delay(500);
				Stop();
			}
			else if (R < 10)
			{
				Stop();
			}
		}
		else if (value == '*')
		{
			Stop();
		}
	}
}

long ultrasonic()
{
	digitalWrite(Trig, LOW);
	delayMicroseconds(4);
	digitalWrite(Trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(Trig, LOW);
	long t = pulseIn(Echo, HIGH);
	bs.print("t = ");
	bs.println(t);

	long cm = t / 29 / 2; // time convert distance
	bs.print(F("cm = "));
	bs.println(cm);
	return cm;
}

void forward()
{
	M1.run(FORWARD);
	M2.run(FORWARD);
	M3.run(FORWARD);
	M4.run(FORWARD);
}
void backward()
{
	M1.run(BACKWARD);
	M2.run(BACKWARD);
	M3.run(BACKWARD);
	M4.run(BACKWARD);
}
void left()
{
	M1.run(BACKWARD);
	M2.run(BACKWARD);
	M3.run(FORWARD);
	M4.run(FORWARD);
}
void right()
{
	M1.run(FORWARD);
	M2.run(FORWARD);
	M3.run(BACKWARD);
	M4.run(BACKWARD);
}
void Stop()
{
	M1.run(RELEASE);
	M2.run(RELEASE);
	M3.run(RELEASE);
	M4.run(RELEASE);
}
long rightsee()
{
	servo.write(20);
	delay(800);
	Left = ultrasonic();
	return Left;
}

long leftsee()
{
	servo.write(180);
	delay(800);
	Right = ultrasonic();
	return Right;
}

void gpsControl()
{
	while (Serial.available() > 0)
	{
		if (gps.encode(Serial.read()))
			displayInfo();
	}

	if (millis() > 5000 && gps.charsProcessed() < 10)
	{
		bs.println(F("No GPS detected: check wiring."));
		// while (true)
		//   ;
	}
}

bool isLatitudeReached(double current_lat)
{
	return abs(current_lat - dest_lat) < THRESHOLD;
}

bool isLongitudeReached(double current_lon)
{
	return abs(current_lon - dest_lng) < THRESHOLD;
	;
}

void displayInfo()
{
	//  bs.print(F("Location: "));
	if (gps.location.isValid())
	{
		//    bs.print(gps.location.lat(), 6);
		lat_sum += gps.location.lat();
		//    bs.print(F(","));
		//    bs.print(gps.location.lng(), 6);
		lon_sum += gps.location.lng();
		cnt += 1;
		if ((cnt & 15) == 10)
		{
			bs.println(F("Average latitude longitude is "));
			double current_lat = lat_sum / 10.0;
			double current_lon = lon_sum / 10.0;

			// double rounded_latitude = round(current_lat * 1000.0) / 1000.0;
			// double rounded_longitude = round(current_lon * 1000.0) / 1000.0;

			// double rounded_dest_latitude = round(dest_lat * 1000.0) / 1000.0;
			// double rounded_dest_longitude = round(dest_lng * 1000.0) / 1000.0;

			bs.print(current_lat, 6);
			bs.print(F(","));
			bs.println(current_lon, 6);

			if (!(isLatitudeReached(current_lat) && isLongitudeReached(current_lon) && params.dest_reached))
				set_route_to_dest(current_lat, current_lon, dest_lat, dest_lng);
			else
				bs.println(F("Already reached destination"));
			reset_location_vars();
		}
	}
	else
	{
		bs.println(F("Satellite Not Locked"));
	}
}

void set_route_to_dest(double current_lat, double current_lon, double dest_lat, double dest_lng)
{
	bs.println(F("Finding route..."));
	if (isLatitudeReached(current_lat))
	{
		if ((cnt & 240) == 0)
			takeTurn(current_lon, dest_lng);
		adjustLongitude(current_lon, dest_lng);
	}
	else
	{
		bool need_to_turn = adjustLatitude(current_lat, dest_lat);
		if (need_to_turn)
			takeTurn(current_lon, dest_lng);
	}
}

void takeTurn(double current_lon, double dest_lng)
{
	bs.println(F("Turning..."));
	display_src_dest(current_lon, dest_lng);
	params.no_of_turns_taken += 1;
	if (cnt & 240)
		return;
	cnt = cnt | 240;
	if ((dest_lng - current_lon) > THRESHOLD)
	{
		bs.println(F("Right turn"));
		right();
		delay(900);
		Stop();
		delay(200);
	}
	else if ((current_lon - dest_lng) > THRESHOLD)
	{
		bs.println(F("Left turn"));
		left();
		delay(900);
		Stop();
		delay(200);
	}
	else
	{
		bs.println(F("Destination reached"));
		Stop();
		delay(200);
	}
}

bool adjustLatitude(double current_lat, double dest_lat)
{
	bs.println(F("Trying to adjust latitude..."));
	display_src_dest(current_lat, dest_lat);
	if ((dest_lat - current_lat) > THRESHOLD)
	{
		bs.println(F("Destination ahead; moving forward"));
		forward();
		delay(100);
		return false;
	}
	else if ((current_lat - dest_lat) > THRESHOLD)
	{
		bs.println(F("Destination behind; moving backward"));
		backward();
		delay(100);
		return false;
	}
	else
	{
		bs.println(F("Latitude reached, taking turn"));
		return true;
	}
}

void adjustLongitude(double current_lon, double dest_lng)
{
	bs.println(F("Trying to adjust longitude..."));
	display_src_dest(current_lon, dest_lng);
	if (!isLongitudeReached(current_lon))
	{
		bs.println(F("Dest Latitude reached, moving forward"));
		forward();
		delay(200);
	}
	else
	{
		bs.println(F("Destination reached"));
		params.dest_reached = true;
		Stop();
		delay(200);
	}
}

void display_src_dest(double src, double dest)
{
	bs.print(src, 6);
	bs.print(F("->"));
	bs.println(dest, 6);
}

void reset_location_vars()
{
	lat_sum = 0.0;
	lon_sum = 0.0;
	cnt = cnt & 240;
}
