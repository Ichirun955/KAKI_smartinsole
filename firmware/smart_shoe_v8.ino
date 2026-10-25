/** 
 * XIAO Smart Shoe - Firmware v8.0 (Live Streaming with Stability Tracking) 
 * ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
 * Features: 
 *   - No SD Card logging (Live BLE stream only) 
 *   - Battery percentage added as field [22] 
 *   - Packet Sequence added as field [23] for UI stability tracking 
 *   - Sends data at 100 Hz 
 */ 
 
#include <Wire.h> 
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h> 
#include <Adafruit_MPR121.h> 
#include <bluefruit.h> 
#include <LPS.h> 
#include <SimpleKalmanFilter.h> 
 
// ÄÄ BLE ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
#define BLE_DEV_NAME "XIAO_Smart_Shoe" 
BLEUart bleuart; 
 
// ÄÄ IMU ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); 
bool bno_active = false; 
 
// ÄÄ MPR121 ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
Adafruit_MPR121 cap = Adafruit_MPR121(); 
bool mpr_active = false; 
 
const int NUM_SENSORS  = 10; 
const int SENSITIVITY  = 25;  
 
int  fixedBaseline[NUM_SENSORS]; 
bool isLocked[NUM_SENSORS]; 
int  unlockCounter[NUM_SENSORS];  
int  pressure[NUM_SENSORS]; 
 
#define UNLOCK_HYSTERESIS 5 
#define GROUND_MIN_SENSORS 2 
 
// ÄÄ Barometer ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
LPS ps; 
SimpleKalmanFilter kalmanFilter(0.1, 0.1, 0.03); 
float offsetAltitude = 0; 
bool  ps_active      = false; 
 
// ÄÄ Timing & Sequence ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
#define TICK_US 10000UL   // 10 ms = 100 Hz 
unsigned long lastTick_us  = 0; 
uint8_t       calibPacketDiv = 0; 
uint32_t      packetSeq = 0; // NEW: Sequence tracker 
 
// ÄÄ Battery ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
#ifndef PIN_VBAT 
  #define PIN_VBAT  31  
#endif 
#define VBAT_ENABLE_PIN 14 
#define BAT_VDIV_RATIO  (510.0f / (1000.0f + 510.0f)) 
#define BAT_ADC_REF     3.6f      
#define BAT_ADC_MAX     4095.0f   
#define BAT_V_FULL      4.2f      
#define BAT_V_EMPTY     3.0f      
#define BAT_SAMPLE_TICKS 100      
 
int  batteryPct      = -1;    
uint8_t batTickDiv   = 0; 
float global_v_bat   = 0.0f; 
 
// ÄÄ BLE connection callback ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
void connect_callback(uint16_t conn_handle) { 
    BLEConnection* conn = Bluefruit.Connection(conn_handle); 
    if (conn) { 
        Serial.println("Connected! Requesting 15ms interval..."); 
        conn->requestConnectionParameter(12); 
    } 
} 
 
void startAdv() { 
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE); 
    Bluefruit.Advertising.addTxPower(); 
    Bluefruit.Advertising.addService(bleuart); 
    Bluefruit.Advertising.addName(); 
    Bluefruit.Advertising.restartOnDisconnect(true); 
    Bluefruit.Advertising.setInterval(32, 244); 
    Bluefruit.Advertising.setFastTimeout(30); 
    Bluefruit.Advertising.start(0); 
} 
 
// ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
void setup() { 
    Serial.begin(115200); 
 
    Serial.println("XIAO Smart Shoe v8.0 - Initialising..."); 
 
    // 1 ÄÄ IMU ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    if (!bno.begin()) { 
        Serial.println("[ERR] BNO055 not found!"); 
    } else { 
        bno_active = true; 
        bno.setMode(OPERATION_MODE_CONFIG); 
        delay(100);  
        Wire.beginTransmission(0x28); 
        Wire.write(0x41);           
        Wire.write(0x18);           
        Wire.endTransmission(); 
        delay(10); 
        Wire.beginTransmission(0x28); 
        Wire.write(0x42);           
        Wire.write(0x00);           
        Wire.endTransmission(); 
        delay(100); 
        bno.setMode(OPERATION_MODE_NDOF); 
        delay(100); 
        bno.setExtCrystalUse(true); 
        Serial.println("[OK] BNO055 active"); 
    } 
 
    // 2 ÄÄ Barometer ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    if (!ps.init()) { 
        Serial.println("[ERR] LPS barometer failed!"); 
    } else { 
        ps_active = true; 
        ps.enableDefault(); 
        ps.writeReg(0x10, 0x50);   
 
        Serial.println("Calibrating altitude zero-point (hold still)..."); 
        delay(500); 
        offsetAltitude = 0; 
        for (int i = 0; i < 50; i++) { 
            offsetAltitude += ps.pressureToAltitudeMeters(ps.readPressureMillibars()); 
            delay(15); 
        } 
        offsetAltitude /= 50.0f; 
        Serial.print("[OK] Barometer active. Offset: "); 
        Serial.println(offsetAltitude, 3); 
    } 
 
    // 3 ÄÄ MPR121 ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    if (!cap.begin(0x5A)) { 
        Serial.println("[ERR] MPR121 not found at 0x5A!"); 
    } else { 
        mpr_active = true; 
 
        for (int i = 0; i < NUM_SENSORS; i++) { 
            fixedBaseline[i]  = 0; 
            isLocked[i]       = false; 
            unlockCounter[i]  = 0; 
            pressure[i]       = 0; 
        } 
 
        cap.writeRegister(MPR121_SOFTRESET, 0x63); 
        delay(200); 
        cap.writeRegister(MPR121_ECR,  0x00); 
        cap.writeRegister(MPR121_MHDR, 0x01); 
        cap.writeRegister(MPR121_NHDR, 0x01); 
        cap.writeRegister(MPR121_NCLR, 0x01); 
        cap.writeRegister(MPR121_MHDF, 0x01); 
        cap.writeRegister(MPR121_NHDF, 0x05); 
        cap.writeRegister(MPR121_NCLF, 0xFF); 
        cap.setThresholds(12, 6); 
        cap.writeRegister(MPR121_ECR,  0x8F); 
 
        Serial.println("[OK] MPR121 active"); 
    } 
 
    // 4 ÄÄ Bluetooth ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); 
    Bluefruit.begin(); 
    Bluefruit.autoConnLed(true); 
    Bluefruit.setTxPower(4); 
    Bluefruit.setName(BLE_DEV_NAME); 
    Bluefruit.Periph.setConnectCallback(connect_callback); 
 
    bleuart.begin(); 
    startAdv(); 
 
    Serial.println("[BLE] Advertising started."); 
 
    // 5 ÄÄ Battery ADC ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    pinMode(VBAT_ENABLE_PIN, OUTPUT);        // <--- ADD THIS 
    digitalWrite(VBAT_ENABLE_PIN, HIGH);     // <--- ADD THIS (Starts turned OFF) 
    pinMode(PIN_VBAT, INPUT); 
    analogReadResolution(12); 
    Serial.println("[OK] Battery ADC ready"); 
} 
 
 
// ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
void loop() { 
    if (!Bluefruit.connected()) { 
        delay(10); 
        return; 
    } 
 
    unsigned long now_us = micros(); 
    if (now_us - lastTick_us < TICK_US) return; 
    lastTick_us = now_us; 
 
    // ÄÄ Increment Sequence Number ÄÄ 
    packetSeq++; 
 
    // ÄÄ 1. IMU ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    imu::Quaternion    q(1.0, 0.0, 0.0, 0.0); 
    imu::Vector<3>    la(0.0, 0.0, 0.0); 
    imu::Vector<3>   gyr(0.0, 0.0, 0.0); 
 
    if (bno_active) { 
        q   = bno.getQuat(); 
        la  = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 
        gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); 
    } 
 
    // ÄÄ 2. Barometer ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    float estimatedAltitude = 0.0; 
    if (ps_active) { 
        float rawAlt   = ps.pressureToAltitudeMeters(ps.readPressureMillibars()) - offsetAltitude; 
        estimatedAltitude = kalmanFilter.updateEstimate(rawAlt); 
    } 
 
    // ÄÄ 3. MPR121 ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    int groundSensorCount = 0; 
 
    if (mpr_active) { 
        for (int i = 0; i < NUM_SENSORS; i++) { 
            int filtered = cap.filteredData(i); 
            int currentBaseline = cap.baselineData(i); 
 
            if (!isLocked[i]) { 
                fixedBaseline[i] = currentBaseline; 
            } 
 
            int drop = fixedBaseline[i] - filtered; 
            if (drop < 0) drop = 0; 
 
            int press_val   = map(drop, 0, SENSITIVITY, 0, 100); 
            pressure[i]     = constrain(press_val, 0, 100); 
 
            if (pressure[i] > 5) { 
                isLocked[i]      = true; 
                unlockCounter[i] = 0;   
                groundSensorCount++; 
            } else { 
                if (isLocked[i]) { 
                    unlockCounter[i]++; 
                    if (unlockCounter[i] >= UNLOCK_HYSTERESIS) { 
                        isLocked[i]      = false; 
                        unlockCounter[i] = 0; 
                    } 
                } 
            } 
        } 
    } 
 
    int groundStatus = (groundSensorCount >= GROUND_MIN_SENSORS) ? 1 : 0; 
 
    // ÄÄ 4. Battery ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    batTickDiv++; 
    if (batTickDiv >= BAT_SAMPLE_TICKS) { 
        batTickDiv = 0; 
 
        // FORCE the switch to turn on and wait for ghost voltage to drain 
        pinMode(14, OUTPUT); 
        digitalWrite(14, LOW);  
        delay(2); // Wait 2 full milliseconds 
 
        uint32_t raw = 0; 
        for (int i = 0; i < 4; i++) { 
            raw += analogRead(PIN_VBAT); 
            delay(1); 
        } 
        raw /= 4; 
 
        digitalWrite(14, HIGH); // Turn the switch back off 
 
        // Print the raw values to the Serial Monitor so we can see the truth! 
        Serial.print("RAW ADC: "); Serial.println(raw); 
 
        float v_pin = (raw / BAT_ADC_MAX) * BAT_ADC_REF; 
       // Change this: 
        // float v_bat = v_pin / BAT_VDIV_RATIO; 
ECHO is on.
        // TO THIS: 
        global_v_bat = v_pin / BAT_VDIV_RATIO; 
ECHO is on.
        Serial.print("V_BAT:   "); Serial.println(global_v_bat); 
 
        int pct = (int)(((global_v_bat - BAT_V_EMPTY) / (BAT_V_FULL - BAT_V_EMPTY)) * 100.0f); 
ECHO is on.
        batteryPct = constrain(pct, 0, 100); 
 
        char bpkt[16]; 
        snprintf(bpkt, sizeof(bpkt), "B:%%d\n", batteryPct); 
        bleuart.print(bpkt); 
    } 
 
    // ÄÄ 5. Send 100 Hz "F" packet ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    char pkt[240];   
    snprintf(pkt, sizeof(pkt), 
        "F:%%.4f,%%.4f,%%.4f,%%.4f,"   // q w x y z 
        "%%.4f,%%.4f,%%.4f,"          // la x y z 
        "%%.4f,%%.4f,%%.4f,"          // gyr x y z  (rad/s) 
        "%%d,%%d,%%d,%%d,%%d,"          // pressure 0-4 
        "%%d,%%d,%%d,%%d,%%d,"          // pressure 5-9 
        "%%d,%%.3f,%%.2f,%%lu\n",        // groundStatus, altitude, batteryPct, packetSeq 
        q.w(), q.x(), q.y(), q.z(), 
        la.x(), la.y(), la.z(), 
        gyr.x(), gyr.y(), gyr.z(), 
        pressure[0], pressure[1], pressure[2], pressure[3], pressure[4], 
        pressure[6], pressure[8], pressure[9], pressure[7], pressure[5], 
        groundStatus, 
        estimatedAltitude, 
        global_v_bat, 
        (unsigned long)packetSeq 
    ); 
    bleuart.print(pkt); 
 
    // ÄÄ 6. Calibration packet at 10 Hz ÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄ 
    calibPacketDiv++; 
    if (calibPacketDiv >= 10) { 
        calibPacketDiv = 0; 
        uint8_t sys = 0, gyro = 0, accel = 0, mag = 0; 
        if (bno_active) bno.getCalibration(&sys, &gyro, &accel, &mag); 
        char cpkt[32]; 
        snprintf(cpkt, sizeof(cpkt), "C:%%d,%%d,%%d,%%d\n", sys, gyro, accel, mag); 
        bleuart.print(cpkt); 
    } 
} 
