int segPins[7] = { A0, A1, A2, A3, A4, A5, 8 };

int num[10][7] = {
  { 1, 1, 1, 1, 1, 1, 0 },  //0
  { 0, 1, 1, 0, 0, 0, 0 },  //1
  { 1, 1, 0, 1, 1, 0, 1 },  //2
  { 1, 1, 1, 1, 0, 0, 1 },  //3
  { 0, 1, 1, 0, 0, 1, 1 },  //4
  { 1, 0, 1, 1, 0, 1, 1 },  //5
  { 1, 0, 1, 1, 1, 1, 1 },  //6
  { 1, 1, 1, 0, 0, 0, 0 },  //7
  { 1, 1, 1, 1, 1, 1, 1 },  //8
  { 1, 1, 1, 1, 0, 1, 1 }   //9
};

void setup() {
  for (int i = 0; i < 7; i++) {
    pinMode(segPins[i], OUTPUT);
  }
  pinMode(1, INPUT);
  pinMode(9, OUTPUT);
}

int count = 0;
void loop() {
  digitalWrite(9, HIGH);
  while (digitalRead(1) == HIGH) ;
    delay(40);
    if (digitalRead(1) == HIGH) {
      count++;
      if (count > 9) {
        count = 0;
      }
    }

  for (int i = 0; i < 7; i++) {
    digitalWrite(segPins[i], num[count][i]);
  }
}