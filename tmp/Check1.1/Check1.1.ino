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
};

void setup()
{
  for (int i=0;i<8;i++){
    pinMode(sw[i],INPUT);
  }

  for (int i=0;i<7;i++){
    pinMode(segPins[i],OUTPUT);
  }
}

void loop() {
  int count = 0;

  for (int i=0;i<8;i++){
    if (digitalRead(sw[i]) == HIGH) {
      count ++;
    }
  }
  
  for (int i=0;i<7;i++){
    digitalWrite(segPins[i],num[count][i] ? HIGH : LOW);
  }

}