#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_HS300x.h>

// Thresholds
const float TEMP_CHANGE_THRESHOLD = 1.0; // 1 celcius degree above baseline
const float HUM_CHANGE_THRESHOLD = 5.0; // 2 % above baseline
const float MAG_THRESHOLD = 100.0; // deviation from baseline
const int LIGHT_CHANGE_THRESHOLD = 30; // clear count change from baseline

// Baseline state (set during first N readings, then locked)
const int   BASELINE_SAMPLES = 10;          // readings to average for baseline
static int  baselineCount    = 0;
static bool baselineLocked   = false;

static float baselineHumid = 0.0;
static float baselineTemp  = 0.0;
static float baselineMag   = 0.0;
static int   baselineClear = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);

  // APDS-9960 set up
  if(!APDS. begin()) {
    Serial.println("Failed to initialize APDS9960 sensor.");
    while (1);
  }

  // IMU setup
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU."); 
    while (1);
  }

  // HS300 setup
  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity/temperature sensor.");
    while (1);
  }

  Serial.println("Sensors ready.");
  Serial.println("Collecting baseline — keep environment still...");
}

// classify from four booleans
const char* classify(bool humidSpike, bool tempSpike,
                     bool magDisturbance, bool lightChange) {
  if (magDisturbance)              return "MAGNETIC_DISTURBANCE_EVENT";
  if (lightChange)                 return "LIGHT_OR_COLOR_CHANGE_EVENT";
  if (humidSpike || tempSpike)     return "BREATH_OR_WARM_AIR_EVENT";
  return "BASELINE_NORMAL";
}

void loop() {
  // IMU MAG
  static float mx = 0.0, my = 0.0, mz = 0.0;
  static float mag = 0.0;           // magnitude of mag vector
  static bool  magDisturbance = false;

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    mag = sqrt(mx*mx + my*my + mz*mz);
    if (baselineLocked) {
      magDisturbance = fabs(mag - baselineMag) > MAG_THRESHOLD;
    }
  }

  // HUMIDITY + TEMP
  static float rh = 0.0;
  static float temp  = 0.0;
  static bool  humidSpike = false;
  static bool  tempSpike  = false;

  float newRH = HS300x.readHumidity();
  float newTemp = HS300x.readTemperature();
  if (newRH > 0.0 && newTemp > -40.0) {
    rh = newRH;
    temp = newTemp;
    if (baselineLocked) {
      humidSpike = (rh - baselineHumid) > HUM_CHANGE_THRESHOLD;
      tempSpike  = (temp  - baselineTemp)  > TEMP_CHANGE_THRESHOLD;
    }
  }

  // LIGHT
  static int  redVal   = 0;
  static int  greenVal = 0;
  static int  blueVal  = 0;
  static int  clearVal = 0;
  static bool lightChange = false;

  if (APDS.colorAvailable()) {
    int nr, ng, nb, nc;
    APDS.readColor(nr, ng, nb, nc);
    if (nc > 0) {
      if (baselineLocked) {
        lightChange = abs(nc - baselineClear) > LIGHT_CHANGE_THRESHOLD;
        baselineClear = nc;   // rolling baseline — detects sudden changes
      }
      redVal   = nr;
      greenVal = ng;
      blueVal  = nb;
      clearVal = nc;
    }
  }

  // BASELINE COLLECTION (first BASELINE_SAMPLES valid cycles)
  if (!baselineLocked && rh > 0.0 && mag > 0.0 && clearVal > 0) {
    baselineHumid += rh;
    baselineTemp += temp;
    baselineMag += mag;
    baselineClear += clearVal;
    baselineCount++;

    Serial.print("Calibrating ");
    Serial.print(baselineCount);
    Serial.print("/");
    Serial.println(BASELINE_SAMPLES);

    if (baselineCount >= BASELINE_SAMPLES) {
      baselineHumid /= BASELINE_SAMPLES;
      baselineTemp /= BASELINE_SAMPLES;
      baselineMag  /= BASELINE_SAMPLES;
      baselineClear /= BASELINE_SAMPLES;
      baselineLocked = true;
      Serial.println("Baseline locked. Monitoring started.");
    }

    delay(500);
    return;   // don't classify until baseline is ready
  }

  // CLASSIFY 
  const char* label = classify(humidSpike, tempSpike, magDisturbance, lightChange);

  // SERIAL OUTPUT
  // Line 1: raw values
  Serial.print("raw,rh=");    Serial.print(rh, 2);
  Serial.print(",temp=");     Serial.print(temp, 2);
  Serial.print(",mag=");    Serial.print(mag, 2);
  Serial.print(",r=");       Serial.print(redVal);
  Serial.print(",g=");       Serial.print(greenVal);
  Serial.print(",b=");       Serial.print(blueVal);
  Serial.print(",clear=");      Serial.println(clearVal);

  // Line 2: binary flags (0 or 1)
  Serial.print("flags,humid_jump="); Serial.print(humidSpike ? 1 : 0);
  Serial.print(",temp_rise=");       Serial.print(tempSpike  ? 1 : 0);
  Serial.print(",mag_shift=");     Serial.print(magDisturbance ? 1 : 0);
  Serial.print(",light_or_color_change=");       Serial.println(lightChange ? 1 : 0);

  // Line 3: final situation label
  Serial.print("event,");
  Serial.println(label);

  delay(500);
}
