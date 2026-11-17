/*
 * Moon Road Gen 4 flight computer: signal connections (Pico GPIO / physical pin)
 *
 * SPI0 (LoRa + NAND shared)  SCK GP6 (9) | MOSI GP3 (5) | MISO GP0 (1)
 *   LoRa                     NSS GP5 (7) | RESET GP9 (12) | DIO0 GP7 (10)
 *   NAND 512MB               CS  GP1 (2)
 *
 * SPI1 (BMP388)              SCK GP10 (14) | SDI/SDA GP11 (15) | SDO GP12 (16) | CS GP13 (17)
 *
 * I2C1 (BMI088 IMU, J1)      SDA GP26 (31) | SCL GP27 (32) | Accel 0x19 | Gyro 0x69
 *
 * J2 (PDB connector)         PWM_Pitch GP18 (24) | PWM_Yaw GP19 (25) | Batt_Monitor GP28/ADC2 (34)
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <LoRa.h>
#include <SDFS.h>
#include <Servo.h>
#include <tusb.h>

// --- Configuration Toggles ---
#define ENABLE_IMU         1
#define ENABLE_BARO        1
#define ENABLE_LORA        1
#define ENABLE_SERVOS      1 
#define ENABLE_DUAL_AXIS   0  // 1 = Pitch & Yaw, 0 = Pitch only (Yaw centered)

// --- Pin Assignments (Moon Road Gen 4 schematic) ---
const uint8_t PIN_SPI_SCK   = 6;   // SPI0_SCK
const uint8_t PIN_SPI_MOSI  = 3;   // SPI0_MOSI
const uint8_t PIN_SPI_MISO  = 0;   // SPI0_MISO
const uint8_t PIN_LORA_CS   = 5;   // CS_LORA
const uint8_t PIN_LORA_RST  = 9;   // RESET
const uint8_t PIN_LORA_DIO0 = 7;   // DIO0
const uint8_t PIN_SD_CS     = 1;   // CS_NAND

const uint8_t PIN_BARO_SCK  = 10;  // SPI1_SCK
const uint8_t PIN_BARO_MOSI = 11;  // SPI1_SDA (BMP388 SDI)
const uint8_t PIN_BARO_MISO = 12;  // SPI1_SDO (BMP388 SDO)
const uint8_t PIN_BARO_CS   = 13;  // CS_BMP

const uint8_t PIN_IMU_SDA   = 26;  // I2C1_SDA
const uint8_t PIN_IMU_SCL   = 27;  // I2C1_SCL
const uint8_t ADDR_ACCEL    = 0x19;
const uint8_t ADDR_GYRO     = 0x69;

const uint8_t PIN_SERVO_PITCH = 18; // PWM_Pitch
const uint8_t PIN_SERVO_YAW   = 19; // PWM_YAW

// --- System Parameters ---
const long  LORA_FREQUENCY    = 868E6;
const int   LORA_SPREADING    = 7;
const long  LORA_BANDWIDTH    = 125000L;
const int   LORA_CODING_RATE  = 5;
const int   LORA_SYNC_WORD    = 0x12;
const int   LORA_PREAMBLE_LEN = 8;
const int   LORA_TX_POWER     = 2;

const float ACCEL_SCALE_G     = 24.0f;
const float GYRO_SCALE_DPS    = 2000.0f;

const uint32_t LOOP_INTERVAL_US   = 10000;  // 100 Hz
const uint32_t BARO_INTERVAL_MS   = 50;     // 20 Hz
const uint32_t BARO_WINDOW_US     = 2000;   
const uint32_t LORA_INTERVAL_MS   = 1000;   // 1 Hz
const uint32_t LORA_TX_GUARD_MS   = 250;    // longer packet, about 170 ms on air
const uint32_t SERIAL_INTERVAL_MS = 250;    // 4 Hz
const uint32_t SD_FLUSH_INTERVAL  = 1000;   
const uint32_t SD_START_DELAY_MS  = 5000;   
const uint32_t BOOT_SETTLE_MS     = 250;

const int SERVO_CENTER = 90;
const int SERVO_LIMIT_MIN = 0;
const int SERVO_LIMIT_MAX = 180;
const int SERVO_PULSE_MIN = 700;
const int SERVO_PULSE_MAX = 2300;

const float MAX_ALLOWABLE_GAIN = 10.0f;

// --- Global State & Hardware Objects ---
Adafruit_BMP3XX barometer;
#if ENABLE_SERVOS
Servo servoPitch;
Servo servoYaw;
#endif
File logFileHandle;

uint32_t lastImuUs      = 0;
uint32_t lastBaroMs     = 0;
uint32_t lastLoraMs     = 0;
uint32_t loraTxStartMs  = 0;
uint32_t lastSerialMs   = 0;
uint32_t lastFlushMs    = 0;
uint32_t baroDurationUs = 0;
uint32_t pidLoopUs      = 0;   // PID block execution time

float currentPressureHpa = 0.0f;
float basePressureHpa = 1013.25f;

float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
float gyroOffsetX = 0.0f, gyroOffsetY = 0.0f, gyroOffsetZ = 0.0f;

bool isSdOnline = false;
bool isLogOpen = false;
bool hasNewLogData = false;
bool isArmed = false;

// --- Controller Gains & States ---
float gainKp = 0.5f;
float gainKi = 0.0f;
float gainKd = 0.02f;

float targetSetpoint = 0.0f;
float pitchIntegral = 0.0f;
float prevPitchError = 0.0f;
float yawIntegral = 0.0f;
float prevYawError = 0.0f;

float lastDeltaTime = 0.0f;
float currentTermP = 0.0f;
float currentTermI = 0.0f;
float currentTermD = 0.0f;
int pitchCommand = SERVO_CENTER;
int yawCommand = SERVO_CENTER;

// --- Command Buffer ---
char rxBuffer[32];
int rxBufferLen = 0;

// --- BMI088 Low-Level Drivers ---
#if ENABLE_IMU
bool writeI2c(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    Wire1.write(val);
    return Wire1.endTransmission() == 0;
}

bool readI2c(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom(addr, len) != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire1.read();
    return true;
}

bool initImu() {
    uint8_t chipId = 0;

    if (!readI2c(ADDR_ACCEL, 0x00, &chipId, 1)) return false;
    if (chipId != 0x1E) {
        Serial.print("Invalid Accel ID: 0x"); Serial.println(chipId, HEX);
        return false;
    }
    writeI2c(ADDR_ACCEL, 0x7E, 0xB6); delay(2);
    writeI2c(ADDR_ACCEL, 0x7C, 0x00); delay(5);
    writeI2c(ADDR_ACCEL, 0x7D, 0x04); delay(50);
    writeI2c(ADDR_ACCEL, 0x41, 0x03);
    writeI2c(ADDR_ACCEL, 0x40, 0xA8);

    if (!readI2c(ADDR_GYRO, 0x00, &chipId, 1)) return false;
    if (chipId != 0x0F) {
        Serial.print("Invalid Gyro ID: 0x"); Serial.println(chipId, HEX);
        return false;
    }
    writeI2c(ADDR_GYRO, 0x14, 0xB6); delay(30);
    writeI2c(ADDR_GYRO, 0x0F, 0x00);
    writeI2c(ADDR_GYRO, 0x10, 0x07);
    writeI2c(ADDR_GYRO, 0x11, 0x00); delay(10);

    return true;
}

void calibrateGyro() {
    Serial.println("Calibrating gyro... keep platform stationary.");
    const int totalSamples = 500;
    int validSamples = 0;
    float sumX = 0, sumY = 0, sumZ = 0;
    uint8_t rawData[6];

    for (int i = 0; i < totalSamples; i++) {
        if (readI2c(ADDR_GYRO, 0x02, rawData, 6)) {
            sumX += (int16_t)((rawData[1] << 8) | rawData[0]) / 32768.0f * GYRO_SCALE_DPS;
            sumY += (int16_t)((rawData[3] << 8) | rawData[2]) / 32768.0f * GYRO_SCALE_DPS;
            sumZ += (int16_t)((rawData[5] << 8) | rawData[4]) / 32768.0f * GYRO_SCALE_DPS;
            validSamples++;
        }
        delay(2);
    }

    if (validSamples > 0) {
        gyroOffsetX = sumX / validSamples;
        gyroOffsetY = sumY / validSamples;
        gyroOffsetZ = sumZ / validSamples;
        Serial.print("Gyro calibration complete. Samples: "); Serial.println(validSamples);
    } else {
        Serial.println("Error: Gyro calibration failed!");
    }
}
#endif

// --- Command Parser ---
void processCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "ARM") {
        if (!isArmed) {
            pitchIntegral = 0.0f;
            yawIntegral = 0.0f;
            prevPitchError = targetSetpoint - gyroY;
            prevYawError = targetSetpoint - gyroZ;
            isArmed = true;
        }
        Serial.println("SYSTEM ARMED, PID RESET");
    }
    else if (cmd == "DISARM") {
        isArmed = false;
        if (isLogOpen) logFileHandle.flush();
        Serial.println("SYSTEM DISARMED");
    }
    else if (cmd.startsWith("KP:") || cmd.startsWith("KI:") || cmd.startsWith("KD:")) {
        if (isArmed) {
            Serial.println("Disarm before updating gains.");
            return;
        }

        String rawValStr = cmd.substring(3);
        if (rawValStr.length() == 0 || !(isDigit(rawValStr.charAt(0)) || rawValStr.charAt(0) == '.')) {
            Serial.println("Invalid gain format, ignored.");
            return;
        }

        float parsedVal = rawValStr.toFloat();
        if (parsedVal < 0.0f || parsedVal > MAX_ALLOWABLE_GAIN) {
            Serial.println("Gain out of accepted limits, ignored.");
            return;
        }

        if (cmd.startsWith("KP:")) {
            gainKp = parsedVal;
        } else if (cmd.startsWith("KI:")) {
            gainKi = parsedVal;
            pitchIntegral = 0.0f;
            yawIntegral = 0.0f;
        } else {
            gainKd = parsedVal;
        }

        Serial.print("Current Gains [Kp/Ki/Kd]: ");
        Serial.print(gainKp, 3); Serial.print("/");
        Serial.print(gainKi, 3); Serial.print("/");
        Serial.println(gainKd, 3);
    }
    else {
        Serial.print("Unknown command: "); Serial.println(cmd);
    }
}

// --- Setup Routine ---
void setup() {
    delay(BOOT_SETTLE_MS);

    uint32_t bootTimer = millis();
    while (!tud_mounted() && millis() - bootTimer < 1500) {
        delay(10);
    }
    if (tud_mounted()) {
        Serial.begin(115200);
        while (!Serial && millis() - bootTimer < 3000);
    }

    Serial.println("Booting flight computer...");

    #if ENABLE_IMU
    Wire1.setSDA(PIN_IMU_SDA);
    Wire1.setSCL(PIN_IMU_SCL);
    Wire1.begin();
    Wire1.setClock(400000);

    if (initImu()) {
        Serial.println("IMU online.");
        calibrateGyro();
    } else {
        Serial.println("Error: IMU initialization failed.");
    }
    #endif

    SPI.setSCK(PIN_SPI_SCK);
    SPI.setTX(PIN_SPI_MOSI);
    SPI.setRX(PIN_SPI_MISO);
    SPI.begin();

    #if ENABLE_LORA
    pinMode(PIN_LORA_CS, OUTPUT);
    digitalWrite(PIN_LORA_CS, HIGH);
    #endif

    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);

    #if ENABLE_BARO
    SPI1.setSCK(PIN_BARO_SCK);
    SPI1.setTX(PIN_BARO_MOSI);
    SPI1.setRX(PIN_BARO_MISO);
    SPI1.begin();

    pinMode(PIN_BARO_CS, OUTPUT);
    digitalWrite(PIN_BARO_CS, HIGH);
    #endif

    #if ENABLE_LORA
    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
    if (LoRa.begin(LORA_FREQUENCY)) {
        LoRa.setSpreadingFactor(LORA_SPREADING);
        LoRa.setSignalBandwidth(LORA_BANDWIDTH);
        LoRa.setCodingRate4(LORA_CODING_RATE);
        LoRa.setPreambleLength(LORA_PREAMBLE_LEN);
        LoRa.setSyncWord(LORA_SYNC_WORD);
        LoRa.setTxPower(LORA_TX_POWER);
        Serial.println("LoRa radio online.");
    } else {
        Serial.println("Error: LoRa initialization failed.");
    }
    #endif

    #if ENABLE_BARO
    if (barometer.begin_SPI(PIN_BARO_CS, &SPI1)) {
        barometer.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
        barometer.setPressureOversampling(BMP3_NO_OVERSAMPLING);
        barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        Serial.println("Barometer online.");

        float pressureSum = 0.0f;
        int validReads = 0;
        for (int i = 0; i < 10; i++) {
            if (barometer.performReading()) {
                pressureSum += barometer.pressure / 100.0f;
                validReads++;
            }
            delay(20);
        }
        if (validReads > 0) {
            basePressureHpa = pressureSum / validReads;
        }
        Serial.print("Base Pressure: "); Serial.print(basePressureHpa, 2); Serial.println(" hPa");
    } else {
        Serial.println("Error: Barometer initialization failed.");
    }
    #endif

    SDFSConfig sdConfig;
    sdConfig.setCSPin(PIN_SD_CS);
    sdConfig.setSPISpeed(4000000);
    SDFS.setConfig(sdConfig);

    if (SDFS.begin()) {
        Serial.println("SD card filesystem mounted.");
        isSdOnline = true;

        char logFileName[20];
        for (int i = 0; i < 100; i++) {
            sprintf(logFileName, "/log_%02d.csv", i);
            if (!SDFS.exists(logFileName)) break;
        }

        logFileHandle = SDFS.open(logFileName, "w");
        if (logFileHandle) {
            isLogOpen = true;
            logFileHandle.println("time_ms,dt_ms,armed,press_hpa,ax,ay,az,gx,gy,gz,p_term,i_term,d_term,pitch_cmd,yaw_cmd,kp,ki,kd");
            logFileHandle.flush();
            Serial.print("Active log file: "); Serial.println(logFileName);
        } else {
            Serial.println("Error: Failed to create log file.");
        }
    } else {
        Serial.println("Error: SD card mount failed.");
        isSdOnline = false;
    }

    #if ENABLE_SERVOS
    servoPitch.attach(PIN_SERVO_PITCH, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    servoYaw.attach(PIN_SERVO_YAW, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    servoPitch.write(SERVO_CENTER);
    servoYaw.write(SERVO_CENTER);
    #endif

    lastImuUs = micros();
    Serial.println("Initialization complete. System DISARMED.");
}

// --- Main Execution Loop ---
void loop() {
    uint32_t currentTimeMs = millis();
    uint32_t currentMicros = micros();

    // 1. IMU Control Loop (100 Hz)
    #if ENABLE_IMU
    if (currentMicros - lastImuUs >= LOOP_INTERVAL_US) {
        uint32_t pidStartUs = micros();   // start of PID timing

        float deltaTime = (currentMicros - lastImuUs) / 1000000.0f;
        lastImuUs = currentMicros;

        if (deltaTime > 0.05f) deltaTime = 0.05f;

        uint8_t buffer[6];

        if (readI2c(ADDR_ACCEL, 0x12, buffer, 6)) {
            accelX = (int16_t)((buffer[1] << 8) | buffer[0]) / 32768.0f * ACCEL_SCALE_G;
            accelY = (int16_t)((buffer[3] << 8) | buffer[2]) / 32768.0f * ACCEL_SCALE_G;
            accelZ = (int16_t)((buffer[5] << 8) | buffer[4]) / 32768.0f * ACCEL_SCALE_G;
        }

        if (readI2c(ADDR_GYRO, 0x02, buffer, 6)) {
            gyroX = ((int16_t)((buffer[1] << 8) | buffer[0]) / 32768.0f * GYRO_SCALE_DPS) - gyroOffsetX;
            gyroY = ((int16_t)((buffer[3] << 8) | buffer[2]) / 32768.0f * GYRO_SCALE_DPS) - gyroOffsetY;
            gyroZ = ((int16_t)((buffer[5] << 8) | buffer[4]) / 32768.0f * GYRO_SCALE_DPS) - gyroOffsetZ;
        }

        float pitchError = targetSetpoint - gyroY;
        pitchIntegral += pitchError * deltaTime;
        pitchIntegral = constrain(pitchIntegral, -50.0f, 50.0f);
        float pitchDerivative = (pitchError - prevPitchError) / deltaTime;
        prevPitchError = pitchError;

        currentTermP = gainKp * pitchError;
        currentTermI = gainKi * pitchIntegral;
        currentTermD = gainKd * pitchDerivative;
        float pitchOutput = currentTermP + currentTermI + currentTermD;

        float yawError = targetSetpoint - gyroZ;
        yawIntegral += yawError * deltaTime;
        yawIntegral = constrain(yawIntegral, -50.0f, 50.0f);
        float yawDerivative = (yawError - prevYawError) / deltaTime;
        prevYawError = yawError;
        float yawOutput = (gainKp * yawError) + (gainKi * yawIntegral) + (gainKd * yawDerivative);

        if (isArmed) {
            pitchCommand = SERVO_CENTER + (int)round(pitchOutput);
            pitchCommand = constrain(pitchCommand, SERVO_LIMIT_MIN, SERVO_LIMIT_MAX);

            #if ENABLE_DUAL_AXIS
            yawCommand = SERVO_CENTER + (int)round(yawOutput);
            yawCommand = constrain(yawCommand, SERVO_LIMIT_MIN, SERVO_LIMIT_MAX);
            #else
            yawCommand = SERVO_CENTER;
            #endif
        } else {
            pitchCommand = SERVO_CENTER;
            yawCommand = SERVO_CENTER;
        }

        #if ENABLE_SERVOS
        servoPitch.write(pitchCommand);
        servoYaw.write(yawCommand);
        #endif

        pidLoopUs = micros() - pidStartUs;   // end of PID timing

        lastDeltaTime = deltaTime;
        hasNewLogData = true;
    }
    #endif

    // 2. Barometer Task (20 Hz)
    #if ENABLE_BARO
    bool isBaroWindowOpen = true;
    #if ENABLE_IMU
    isBaroWindowOpen = (micros() - lastImuUs < BARO_WINDOW_US);
    #endif

    if ((currentTimeMs - lastBaroMs >= BARO_INTERVAL_MS) && isBaroWindowOpen) {
        lastBaroMs = currentTimeMs;
        uint32_t baroStartMicros = micros();

        if (barometer.performReading()) {
            currentPressureHpa = barometer.pressure / 100.0f;
        }

        baroDurationUs = micros() - baroStartMicros;
    }
    #endif

    // 3. Command Polling (Serial & LoRa)
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();
        if (incomingChar == '\n' || incomingChar == '\r') {
            if (rxBufferLen > 0) {
                rxBuffer[rxBufferLen] = '\0';
                processCommand(String(rxBuffer));
                rxBufferLen = 0;
            }
        } else if (rxBufferLen < 31) {
            rxBuffer[rxBufferLen] = incomingChar;
            rxBufferLen++;
        }
    }

    #if ENABLE_LORA
    if (currentTimeMs - loraTxStartMs >= LORA_TX_GUARD_MS) {
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            String incomingPayload = "";
            while (LoRa.available()) {
                incomingPayload += (char)LoRa.read();
            }
            processCommand(incomingPayload);
        }
    }
    #endif

    // 4. Debug Output Task (4 Hz)
    if (Serial && (currentTimeMs - lastSerialMs >= SERIAL_INTERVAL_MS)) {
        lastSerialMs = currentTimeMs;
        Serial.print(isArmed ? "[ARMED] " : "[DISARMED] ");
        Serial.print("P: "); Serial.print(currentPressureHpa, 2); Serial.print("hPa | ");
        Serial.print("Accel: "); Serial.print(accelX, 2); Serial.print(","); Serial.print(accelY, 2); Serial.print(","); Serial.print(accelZ, 2); Serial.print(" | ");
        Serial.print("Gyro: "); Serial.print(gyroX, 1); Serial.print(","); Serial.print(gyroY, 1); Serial.print(","); Serial.print(gyroZ, 1); Serial.print(" | ");
        Serial.print("PitchCmd: "); Serial.print(pitchCommand); Serial.print(" | ");
        Serial.print("DtMs: "); Serial.print(lastDeltaTime * 1000.0f, 2); Serial.print(" | ");
        Serial.print("PidUs: "); Serial.print(pidLoopUs); Serial.print(" | ");
        Serial.print("BaroUs: "); Serial.println(baroDurationUs);
    }

    // 5. LoRa Telemetry Task (1 Hz)
    #if ENABLE_LORA
    if (currentTimeMs - lastLoraMs >= LORA_INTERVAL_MS) {
        lastLoraMs = currentTimeMs;
        if (LoRa.beginPacket()) {
            LoRa.print("ARM:"); LoRa.print(isArmed ? 1 : 0);
            LoRa.print(",P:");  LoRa.print(currentPressureHpa, 2);
            LoRa.print(",AX:"); LoRa.print(accelX, 2);
            LoRa.print(",AY:"); LoRa.print(accelY, 2);
            LoRa.print(",AZ:"); LoRa.print(accelZ, 2);
            LoRa.print(",GX:"); LoRa.print(gyroX, 1);
            LoRa.print(",GY:"); LoRa.print(gyroY, 1);
            LoRa.print(",GZ:"); LoRa.print(gyroZ, 1);
            LoRa.print(",KP:"); LoRa.print(gainKp, 3);
            LoRa.print(",KI:"); LoRa.print(gainKi, 3);
            LoRa.print(",KD:"); LoRa.print(gainKd, 3);
            LoRa.endPacket(true);
            loraTxStartMs = currentTimeMs;
        }
    }
    #endif

    // 6. SD Card Logging 
    if (isLogOpen && hasNewLogData && (currentTimeMs >= SD_START_DELAY_MS)) {
        hasNewLogData = false;

        logFileHandle.print(currentTimeMs);         logFileHandle.print(",");
        logFileHandle.print(lastDeltaTime * 1000.0f, 2); logFileHandle.print(",");
        logFileHandle.print(isArmed ? 1 : 0);       logFileHandle.print(",");
        logFileHandle.print(currentPressureHpa, 2); logFileHandle.print(",");
        logFileHandle.print(accelX, 2);             logFileHandle.print(",");
        logFileHandle.print(accelY, 2);             logFileHandle.print(",");
        logFileHandle.print(accelZ, 2);             logFileHandle.print(",");
        logFileHandle.print(gyroX, 1);              logFileHandle.print(",");
        logFileHandle.print(gyroY, 1);              logFileHandle.print(",");
        logFileHandle.print(gyroZ, 1);              logFileHandle.print(",");
        logFileHandle.print(currentTermP, 2);       logFileHandle.print(",");
        logFileHandle.print(currentTermI, 2);       logFileHandle.print(",");
        logFileHandle.print(currentTermD, 2);       logFileHandle.print(",");
        logFileHandle.print(pitchCommand);          logFileHandle.print(",");
        logFileHandle.print(yawCommand);            logFileHandle.print(",");
        logFileHandle.print(gainKp, 3);             logFileHandle.print(",");
        logFileHandle.print(gainKi, 3);             logFileHandle.print(",");
        logFileHandle.println(gainKd, 3);
    }

    if (isLogOpen && (currentTimeMs - lastFlushMs >= SD_FLUSH_INTERVAL)) {
        lastFlushMs = currentTimeMs;
        logFileHandle.flush();
    }
}

