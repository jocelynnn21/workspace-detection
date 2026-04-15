void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  static int x = 0;
  int y = (x % 20);
  Serial.println(y);
  x++;
  delay(100);
}
