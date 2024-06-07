/*
  iss-tracker.ino - Main methods
 */

#include <Arduino.h>
#include <string.h>
#include <SPI.h>
#include <Wire.h>
#include "arduino_secrets.h"
#include "coord.h"
#include "orbit_utils.h"
#include "wifi_utils.h"
#include "display_utils.h"
#include "pedestal.h"
#include "elapsedMillis.h"
#include "defs.h"

// Make sure to enter the appropriate info in arduino_secrets.h
//char * ssid = NULL;
//char * pass = NULL;
//char ssid1[] = "TJ1";    // network SSID
//char pass1[] = "rileyhall1";    // network password (use for WPA, or use as key for WEP) 
//char ssid2[] = "WPSGuest";    // network SSID
//char pass2[] = "";    // network password (use for WPA, or use as key for WEP) 

uint8_t ssidCount = 5;
//const char *ssids[] = { "WPS-Guest", "WMS-Guest", "WPSGuest", "WMSGuest", "TJ1" };
//const char *passes[] = { "", "", "", "", "rileyhall1" };

String ssid;
String pass;
String ssids[] = { "WPS-Guest", "WMS-Guest", "WPSGuest", "WMSGuest", "tj1" };
String passes[] = { "", "", "", "", "rileyhall1" };

Vec3 llaRef = {42.53598940025664, -71.14035084369107,42}; // Pedestal Lat/Lon/Altitude(meters)


// Wrapper Structs
NtpQueryHandler ntp{};
TleQueryHandler tle{};
Pedestal ped{};
Orbit orb{};

// Misc. variable declaration
Vec3 posECI, velECI, posECEF, posLLA, posNED, posAER;
double era;
int wifiStatus = WL_IDLE_STATUS;
uint32_t lastTleUpdateMillis, lastOrbitUpdateMillis, lastNtpUpdateMillis;

bool ntpPacketSent = false;
bool tleQuerySent = false;

void setup() {
    // Initialize serial and wait for port to open
    Serial.begin(9600);
    delay(250);
    while (!Serial && WAIT_FOR_SERIAL) {}

    Serial.println("Starting ISS Tracker");

    /* Initialise the display */
    delay(250); // wait for the OLED to power up
    if (!display.begin(0x3C, true)) {  // Address 0x3C default
      Serial.println("No display detected");
      while (CHECK_DISPLAY_CONNECTION) delay(10);
    }

    Serial.println("Do Display");

    // Show image buffer on the display hardware.
    // Since the buffer is intialized with an Adafruit splashscreen
    // internally, this will display the splashscreen.
    display.display();
    delay(1000);
    display.setRotation(1);
    resetDisplay(0,0,1);
    display.println("Starting ISS Tracker");
    display.display();

    Serial.println("Starting Ped");

    // Initialize pedestal wrapper
    resetDisplay(0,0,1);
    display.printf("Compass Cal...");
    display.display();
    ped.begin();

    // Test pointer elevation range
    // Should point at 0 degrees, then -90, then +90, then back to 0
    // If any are significantly misaligned from expectation,
    // then you'll need to adjust the SERVO_MIN_PWM & SERVO_MAX_PWM
    // params in defs.h. Check the spec sheet for your micro-servo of choice

    Serial.println("Point Elevation: -90");
    resetDisplay(0,0,2);
    display.printf("El: -90");
    display.display();
    ped.setElevation(0);
    delay(2000);

    Serial.println("Point Elevation: 90");
    resetDisplay(0,0,2);
    display.printf("El: +90");
    display.display();
    ped.setElevation(180);
    delay(2000);

    resetDisplay(0,0,2);
    display.printf("El: 0");
    display.display();
    Serial.println("Point Elevation: 0");
    ped.setElevation(90);
    delay(2000);

    // If using the compass to align northward, perform a few attempts at automatically pointing northward
    if (!DO_BYPASS_COMPASS) {
        double az = ped.getAverageHeading();
        uint8_t tryCount = 0;
        // loop until we are close
        double heading = 0;
        double diff = 360;
        Serial.printf("North Finding Az Diff: %0.1f\n",fabs(diff));

        while( fabs(diff) > 10 && tryCount < 100 )
        {
            resetDisplay(0,0,1);
            az = ped.getAverageHeading();
            display.printf("North Finding:\nAz Before Point: %0.1f\n", az);
            display.display();
            diff = ped.pointToHeading(heading);
            Serial.printf("Northing Finding Diff: %0.1f\n", diff);
            tryCount++;
            delay(1000);
        }
        resetDisplay(0,0,2);
        display.printf("North Finding Az: %0.1f\n",ped.getAverageHeading());
        display.display();
        Serial.printf("North Finding Az: %0.1f\n",ped.getAverageHeading());
        delay(100);
    }
    

    // Reset stepper step count to zero to establish current step count as zero azimuth
    ped.stepper.setCurrentPosition(0);
    
    Serial.println("Starting WIFI");

    // check for the WiFi module:
    WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
    while (WiFi.status() == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
        resetDisplay(0,0,1);
        display.println("Communication with WiFi module failed!");
        display.display();
        // don't continue
        while (true) {}
    }

    // Check Wifi Co-processor Firmware
    String fv = WiFi.firmwareVersion();
    Serial.print("Found firmware "); Serial.println(fv);

    // List visible Wifi Networks
    Serial.println("Scanning available networks...");
    int numSsid = listNetworks();

    for( int i=0; i < numSsid; ++i )
    {
      for( int j=0; j < ssidCount; ++j )
      {
          Serial.print("Trying WifiSSID: ");
          Serial.print(WiFi.SSID(i));
          Serial.print("  SSID: ");
          Serial.println(ssids[j]);
          
          //if( strcmpi( ssids[j], WiFi.SSID(i) ) )
          if( ssids[j].equalsIgnoreCase(WiFi.SSID(i)) )
          {
            Serial.print("Found SSID Match: ");
            Serial.println(WiFi.SSID(i));
            ssid = WiFi.SSID(i);
            pass = passes[j];
          }
      }
    }

    if( !ssid )
    {
      do
      {
        Serial.println("Unable to find valid Wifi SSID");
        resetDisplay(0,0,1);
        display.println("Unable to find valid Wifi SSID");
        display.display();
        delay(10000);

      } while(true);
    }

    // Attempt to connect to Wifi network:
    resetDisplay(0,10,1);
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    display.print("Attempting to connect to SSID:\n");
    display.println(ssid);
    display.display();

    // Connect to WPA/WPA2 network
    do {
        wifiStatus = WiFi.begin(ssid.c_str(), pass.c_str());
        Serial.print("Wifi Status: ");
        Serial.println(wifiStatus);
        delay(10000);     // wait until connection is ready!
    } while (wifiStatus != WL_CONNECTED);

    Serial.println("Connected to wifi");
    display.println("Connected to wifi");
    display.display();
    delay(1000);

    // Start NTP connection
    ntp.begin();
    resetDisplay(0,10,1);
    Serial.println("\nStarting connection to NTP server...");
    display.println("\nStarting connection to NTP server...");
    display.display();
    delay(100);

    // Get unix time
    ntp.sendNTPpacket(); // send an NTP packet to the time server
    // wait to see if a reply is available
    while (!ntp.parsePacket()) {};
    displayCurrTime(0.0,0.0);

    // Send initial TLE query
    tle.sendQuery();

    // Wait for response
    while (!tle.rcvData()){}
    tle.readTLE();

    Serial.println(tle.line1);
    Serial.println(tle.line2);

    // Parse received TLE
    tle.getOrbit(orb);

    if (DO_PRINT_DEBUG) {
        Serial.println();
        Serial.print("epoch: "); Serial.println(orb.epoch_J);
        Serial.print("utc:   "); Serial.println(orb.epochUTC);
        Serial.print("incl:  "); Serial.println(orb.incl,8);
        Serial.print("a:     "); Serial.println(orb.a);
        Serial.print("ecc:   "); Serial.println(orb.ecc,8);
        Serial.print("Omega: "); Serial.println(orb.Omega,8);
        Serial.print("omega: "); Serial.println(orb.omega,8);
        Serial.print("M0:    "); Serial.println(orb.M0,8);
        Serial.print("n:     "); Serial.println(orb.n,8);
        Serial.print("n_dot: "); Serial.println(orb.n_dot,16);
    }
    
    // Initialize timers
    lastNtpUpdateMillis   = millis();
    lastTleUpdateMillis   = lastNtpUpdateMillis;
    lastOrbitUpdateMillis = lastNtpUpdateMillis;
    delay(100);

    if (DO_PRINT_DEBUG) {
        Serial.print("In setup(): lastNtpUpdateMillis: ");
        Serial.println(int32_t(lastTleUpdateMillis));
        Serial.print("In setup(): lastTleUpdateMillis: ");
        Serial.println(int32_t(lastTleUpdateMillis));
    }
}

uint32_t currMillis, timeSinceNtpUpdate_ms, timeSinceTleUpdate_ms, timeSinceOrbitUpdate_ms;
elapsedMillis repointTime = 0;

void loop() {
    // Get current time
    currMillis = millis();

    // Try to catch overflow of millis() count and handle gracefully
    if (currMillis < lastNtpUpdateMillis) {
        lastNtpUpdateMillis = currMillis;
        lastTleUpdateMillis = currMillis;
        lastOrbitUpdateMillis = currMillis;

        Serial.println("Time counter overflow hit");
    }

    timeSinceNtpUpdate_ms = (currMillis - lastNtpUpdateMillis);
    timeSinceTleUpdate_ms = (currMillis - lastTleUpdateMillis);
    timeSinceOrbitUpdate_ms = (currMillis - lastOrbitUpdateMillis);

    // Advance stepper if necessary
    ped.runStepper();

    // Recheck wifi connection status, and try to reconnect if disconnected
    if (WiFi.status() != WL_CONNECTED) {
        resetDisplay(0,0,1);
        display.println("Wifi disconnected, attempting to reconnect...");
        display.display();
        do {
            wifiStatus = WiFi.begin(ssid.c_str(), pass.c_str());
            delay(10000);     // wait until connection is ready!
        } while (wifiStatus != WL_CONNECTED);
    }

    // Resend NTP packet regularly to keep time synchronized
    if (!ntpPacketSent && (timeSinceNtpUpdate_ms > (TIME_REFRESH_DELAY_MIN*60*1000))) {
        ntp.sendNTPpacket();
        ntpPacketSent = true;
        Serial.println("NTP Packet Sent");
    } else {
        if (ntp.parsePacket()) {
            Serial.println("Parsed NTP Packet");
            lastNtpUpdateMillis = millis();
            ntpPacketSent = false;
        }
    }

    // Resend TLE Query regularly to get updated ephemeris
    if (!tleQuerySent && (timeSinceTleUpdate_ms > (TLE_REFRESH_DELAY_MIN * 60*1000))) {
        tle.sendQuery();
        tleQuerySent = true;
        Serial.println("TLE Query Sent");
    } else if (tleQuerySent) {
        if (tle.rcvData()) {
            Serial.println("Updating Ephemeris");
            tle.readTLE(); // Read received TLE data into separate lines
            tle.getOrbit(orb); // Update orbit from received TLE data
            lastTleUpdateMillis = millis();
            tleQuerySent = false;
        }
    }

    uint64_t currUTC_ms = uint64_t(ntp.unixEpoch)*1000 + uint64_t(timeSinceNtpUpdate_ms);

    // Update Az/El at regular rate
    if (timeSinceOrbitUpdate_ms >= ORBIT_REFRESH_DELAY_MS) {
        
        // Calc Earth-Rotation-Angle for current UTC
        era = getEraFromJulian(getJulianFromUnix(currUTC_ms/1000));
        
        // Calc ECI Pos/Vel for current UTC
        orb.calcPosVelECI_UTC(currUTC_ms,posECI,velECI);
        lastOrbitUpdateMillis = millis();
        posECEF = eci2ecef(posECI,-era);
        posLLA = ecef2lla(posECEF,DEGREES);
        posNED = ecef2ned(posECEF,llaRef,DEGREES);
        posAER = ned2AzElRng(posNED);

        if (DO_PRINT_DEBUG) {
            Serial.printf("System Time: %04i-%02i-%02i  %02i:%02i:%02i\n",
                        year(),month(),day(),hour(),minute(),second());
          
            Serial.printf("timeSinceNtpUpdate_ms: %lu, timeSinceTleUpdate_ms: %lu\n",
                            timeSinceNtpUpdate_ms, timeSinceTleUpdate_ms);
            Serial.print("era:   "); Serial.println(era*RAD_TO_DEG,3);
            Serial.printf("posECI:  [%0.3f,%0.3f,%0.3f]\n",posECI.x,posECI.y,posECI.z);
            Serial.printf("posECEF: [%0.3f,%0.3f,%0.3f]\n",posECEF.x,posECEF.y,posECEF.z);
            Serial.printf("posLLA:  [%0.3f,%0.3f,%0.3f]\n",posLLA.x,posLLA.y,posLLA.z/1e3);
            Serial.printf("posNED:  [%0.3f,%0.3f,%0.3f]\n",posNED.x,posNED.y,posNED.z);
            Serial.printf("posAER:  [%0.3f,%0.3f,%0.3f]\n",posAER.x,posAER.y,posAER.z);
            Serial.printf("curHeading: %0.3f\n", ped.getHeading());
        }

        // Update target azimuth only if reached current step target (to avoid interrupting smooth movement)
        if (ped.stepper.distanceToGo() == 0)
        {
            ped.setTargetAz(posAER[0]);
            Serial.printf("posAER: %0.3f curHeading: %0.3f\n", posAER[0], ped.getHeading());
        }
        ped.setElevation(90+posAER[1]);

        // Display current date/time on screen
        displayCurrTime(posAER[0],posAER[1]);
    }
}
