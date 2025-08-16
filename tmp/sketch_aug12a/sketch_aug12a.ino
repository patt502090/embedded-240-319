int sw[8] = {0,1,2,3,4,5,6,7};
int segPins[7] = {A0,A1,A2,A3,A4,A5,8};

int num[10][7] = {
  {1,1,1,1,1,1,0},//0
  {0,1,1,0,0,0,0},//1
  {1,1,0,1,1,0,1},//2
  {1,1,1,1,0,0,1},//3
  {0,1,1,0,0,1,1},//4
  {1,0,1,1,0,1,1},//5
  {1,0,1,1,1,1,1},//6
  {1,1,1,0,0,0,0},//7
  {1,1,1,1,1,1,1},//8
  {1,1,1,1,0,1,1}//9
};

void setup() {
  for (int i=0;i<8;i++) {
    pinMode(sw[i],INPUT);
  }

  for (int i=0;i<7;i++){
    pinMode(segPins[i],OUTPUT);
  }
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
}

void loop() {
  int result = 0;
  int low4 = PIND & 0x0f;
  int high4 = (PIND >> 4) & 0x0f;
  result = high4 * low4;
  int h = ( result / 100 ) %10;
  int t = ( result / 10 ) %10;
  int u = result  %10;
  if (h > 0){
      digitalWrite(9,LOW);
      digitalWrite(10,LOW);
      digitalWrite(11,LOW);
      digitalWrite(11,HIGH);
    for (int i=0;i<7;i++){
      digitalWrite(segPins[i],num[h][i]);
    }
    delay(5);
  }if (t > 0 || h > 0) {
      digitalWrite(9,LOW);
      digitalWrite(10,LOW);
      digitalWrite(11,LOW);
      digitalWrite(10,HIGH);
    for (int i=0;i<7;i++){
      digitalWrite(segPins[i],num[t][i]);
    }
    delay(5);
  }
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
  digitalWrite(9,HIGH);
  for (int i=0;i<7;i++){
    digitalWrite(segPins[i],num[u][i]);
  }
  delay(5);
}
