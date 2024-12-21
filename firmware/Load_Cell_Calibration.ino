const int pin0 = A0;
const int pin1 = A1;
const int pin2 = A2;
const int pin3 = A3;

int pin0_value = 0;
int pin1_value = 1;
int pin2_value = 2;
int pin3_value = 3;


void setup() {
  Serial.begin(9600);
}

void loop() {
  pin0_value = analogRead(pin0);
  pin1_value = analogRead(pin1);
  pin2_value = analogRead(pin2);
  pin3_value = analogRead(pin3);

  Serial.print(pin0_value);
  Serial.print(",");
  Serial.print(pin1_value);
  Serial.print(",");
  Serial.print(pin2_value);
  Serial.print(",");
  Serial.println(pin3_value);
  delay(1000);

}
