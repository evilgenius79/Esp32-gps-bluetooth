#include <M5StickCPlus2.h>
#include "BluetoothSerial.h"

// Hardware & Race Constants
#define GPS_BAUD 460800
#define GROVE_RX 33
#define GROVE_TX 32
#define G_TRIGGER 0.28      // Sensitivity: higher = harder hit required
#define SPEED_DEADZONE 1.5
#define NMEA_MAX_LEN 128    // Max NMEA sentence length

BluetoothSerial SerialBT;

enum RaceState { IDLE, STAGED, RACING, FINISHED };
RaceState currentState = IDLE;

// Telemetry
float currentSpeedMPH = 0, distanceFeet = 0;
float accX = 0, accY = 0, accZ = 0;
float offsetX = 0; // Calibration offset
unsigned long raceStartTime = 0;
uint32_t msgCount = 0;
uint32_t lastUIUpdate = 0;
bool hasFix = false;

// Race Results
float time60ft = 0, time0_60mph = 0, time1_4mile = 0, trap1_4 = 0;

// FIX 1 (bounds check) + FIX 3 (\r strip): parseGPS now checks i+1 is in range
// and the caller trims the line before passing it in.
void parseGPS(const String& line) {
    if (line.indexOf("RMC") == -1) return;

    int commaCount = 0, startIndex = 0;
    int len = line.length();
    for (int i = 0; i < len; i++) {
        if (line[i] == ',') {
            commaCount++;
            // FIX 1: bounds-check before accessing i+1
            if (commaCount == 2 && i + 1 < len) {
                hasFix = (line[i + 1] == 'A');
            }
            if (commaCount == 7) startIndex = i + 1;
            if (commaCount == 8) {
                float rawMPH = line.substring(startIndex, i).toFloat() * 1.15078f;
                currentSpeedMPH = (rawMPH < SPEED_DEADZONE) ? 0.0f : rawMPH;
                break;
            }
        }
    }
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);

    // Power-on feedback
    M5.Power.setVibration(100);
    delay(100);
    M5.Power.setVibration(0);

    M5.Lcd.setBrightness(200);
    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(BLACK);

    M5.Imu.init();
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GROVE_RX, GROVE_TX);
    SerialBT.begin("M5_DRAG_STAGED");
}

void loop() {
    M5.update();
    M5.Imu.getAccel(&accX, &accY, &accZ);

    // LIVE G-Force relative to calibration
    float liveG = accX - offsetX;

    // FIX 4 + FIX 6: Guard against pressing A mid-race; include trap1_4 in reset
    if (M5.BtnA.wasPressed() && currentState != RACING) {
        // FIX 5: Average 10 IMU samples for a stable calibration zero
        float sumX = 0;
        for (int s = 0; s < 10; s++) {
            float ax, ay, az;
            M5.Imu.getAccel(&ax, &ay, &az);
            sumX += ax;
            delay(10);
        }
        offsetX = sumX / 10.0f;

        currentState = STAGED;
        time60ft = 0; time0_60mph = 0; time1_4mile = 0; trap1_4 = 0; distanceFeet = 0;
    }

    // Launch Trigger (Must be STAGED first)
    if (hasFix && currentState == STAGED && liveG > G_TRIGGER) {
        currentState = RACING;
        raceStartTime = millis();
    }

    // GPS Processing
    static String nmeaLine = "";
    static uint32_t lastGPSTime = 0;  // FIX 2: track actual inter-sentence interval

    while (Serial2.available()) {
        char c = Serial2.read();
        SerialBT.write(c);

        // FIX 7: Reset buffer on sentence start — prevents unbounded growth
        // from missing newlines, and discards any partial previous sentence.
        if (c == '$') {
            nmeaLine = "$";
            msgCount++;
            continue;
        }

        // FIX 7: Cap buffer length to guard against runaway sentences
        if (nmeaLine.length() < NMEA_MAX_LEN) {
            nmeaLine += c;
        }

        if (c == '\n') {
            // FIX 3: Strip trailing \r\n before parsing
            nmeaLine.trim();

            parseGPS(nmeaLine);

            if (currentState == RACING) {
                uint32_t now = millis();
                float elapsed = (now - raceStartTime) / 1000.0f;

                // FIX 2: Use actual time between GPS sentences, not a fixed delta.
                // Cap at 0.5s to limit error if a sentence is briefly lost.
                float dt = (lastGPSTime > 0) ? (now - lastGPSTime) / 1000.0f : 0.0f;
                if (dt > 0.5f) dt = 0.5f;

                distanceFeet += (currentSpeedMPH * 1.46667f) * dt;

                if (distanceFeet >= 60.0f && time60ft == 0)     time60ft = elapsed;
                if (currentSpeedMPH >= 60.0f && time0_60mph == 0) time0_60mph = elapsed;
                if (distanceFeet >= 1320.0f) {
                    time1_4mile = elapsed;
                    trap1_4 = currentSpeedMPH;
                    currentState = FINISHED;
                }
            }

            lastGPSTime = millis();
            nmeaLine = "";
        }
    }

    // Power Off (Side Button)
    if (M5.BtnB.pressedFor(2000)) M5.Power.powerOff();

    // UI Refresh
    if (millis() - lastUIUpdate > 100) {
        M5.Lcd.fillRect(0, 0, 240, 20, (hasFix ? BLUE : RED));
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.setCursor(5, 5);
        M5.Lcd.printf("GPS:%dHz | G:%0.2f", (msgCount * 10), liveG);
        M5.Lcd.setCursor(180, 5);
        M5.Lcd.printf("%d%%", M5.Power.getBatteryLevel());

        M5.Lcd.setCursor(0, 30);
        M5.Lcd.setTextSize(3);
        M5.Lcd.setTextColor(YELLOW, BLACK);
        M5.Lcd.printf("%0.1f MPH ", currentSpeedMPH);

        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(0, 65);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.printf("60ft: %0.2fs  0-60: %0.2fs\n", time60ft, time0_60mph);
        M5.Lcd.printf("1/4: %0.2fs @ %0.1f\n", time1_4mile, trap1_4);

        M5.Lcd.setCursor(0, 115);
        if (currentState == STAGED) {
            // FIX 8: Distinguish between staged-with-fix and staged-waiting-for-fix
            if (hasFix) {
                M5.Lcd.setTextColor(GREEN, BLACK);
                M5.Lcd.print("STAGED: READY FOR HIT ");
            } else {
                M5.Lcd.setTextColor(YELLOW, BLACK);
                M5.Lcd.print("STAGED: WAITING FOR FIX");
            }
        } else if (currentState == RACING) {
            M5.Lcd.setTextColor(RED, BLACK);
            M5.Lcd.printf("GO! DIST: %0.0f ft      ", distanceFeet);
        } else if (currentState == FINISHED) {
            M5.Lcd.setTextColor(CYAN, BLACK);
            M5.Lcd.print("DONE! A TO RE-STAGE    ");
        } else {
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.print("PRESS A TO STAGE RUN   ");
        }

        msgCount = 0;
        lastUIUpdate = millis();
    }
}
