#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <PDM.h>

// Thresholds
const int AUDIO_NOISE_THRESHOLD = 300;   // MAD level
const int LIGHT_BRIGHT_THRESHOLD = 50;   // ambient lux-like counts
const int PROX_NEAR_THRESHOLD    = 100;  // 0–255 (0 = closest)
const float ACCEL_MOVE_THRESHOLD   = 0.15; // g deviation from baseline

// PDM audio buffer 
short sampleBuffer[256];
volatile int samplesRead = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  // PDM setup
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM microphone."); 
    while (1);
  }

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

  Serial.println("Sensors ready.");
}

// compute MAD level
int computeLevel() {
  if (samplesRead == 0) return 0;
  long sum = 0;
  for (int i = 0; i < samplesRead; i++) {
    sum += abs(sampleBuffer[i]);
  }
  int level = sum / samplesRead;
  samplesRead = 0;
  return level;
}

// classify from four booleans
const char* classify(bool sound, bool dark, bool moving, bool near) {
  if (!sound && !dark && !moving && !near) return "QUIET_BRIGHT_STEADY_FAR";
  if ( sound && !dark && !moving && !near) return "NOISY_BRIGHT_STEADY_FAR";
  if (!sound &&  dark && !moving &&  near) return "QUIET_DARK_STEADY_NEAR";
  if ( sound && !dark &&  moving &&  near) return "NOISY_BRIGHT_MOVING_NEAR";
  return "UNKNOWN";
}

void loop() {
  // AUDIO 
  int  mic   = computeLevel();
  bool sound = (mic > AUDIO_NOISE_THRESHOLD);

  // LIGHT (clear channel)
  static int clear = 1;
  static bool dark  = true;
  if (APDS.colorAvailable()) {
    int r, g, b;
    APDS.readColor(r, g, b, clear);
    dark = (clear <= LIGHT_BRIGHT_THRESHOLD);  // dark=1 means NOT bright
  }

  // IMU (accel magnitude deviation)
  float motion = 0.0;
  bool  moving = false;
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    motion = fabs(sqrt(ax*ax + ay*ay + az*az) - 1.0f);
    moving = (motion > ACCEL_MOVE_THRESHOLD);
  }

  // PROXIMITY 
  static int  prox = 0;
  static bool near = true;
  if (APDS.proximityAvailable()) {
    prox = APDS.readProximity();
    near = (prox < PROX_NEAR_THRESHOLD);
  }

  // CLASSIFY 
  const char* label = classify(sound, dark, moving, near);

  // SERIAL OUTPUT
  // Line 1: raw values
  Serial.print("raw,mic=");    Serial.print(mic);
  Serial.print(",clear=");     Serial.print(clear);
  Serial.print(",motion=");    Serial.print(motion, 4);  // 4 decimal places
  Serial.print(",prox=");      Serial.println(prox);

  // Line 2: binary flags (0 or 1)
  Serial.print("flags,sound="); Serial.print(sound ? 1 : 0);
  Serial.print(",dark=");       Serial.print(dark  ? 1 : 0);
  Serial.print(",moving=");     Serial.print(moving ? 1 : 0);
  Serial.print(",near=");       Serial.println(near ? 1 : 0);

  // Line 3: final situation label
  Serial.print("state,");
  Serial.println(label);

  delay(500);
}
