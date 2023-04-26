

void setup() {
  //motor outputs
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);

  analogWrite(7, 1);
  delay(500);
  analogWrite(7, 200);
  delay(500);
  analogWrite(7, 1);
  delay(500);
  analogWrite(7, 150);
}

void loop() {


}
