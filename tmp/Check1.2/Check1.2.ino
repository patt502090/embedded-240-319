int sw[8] = {0,1,2,3,4,5,6,7};
int segPins[7] = {A0,A1,A2,A3,A4,A5,8};


int num[16][7] = {
  {1,1,1,1,1,1,0},//0
  {0,1,1,0,0,0,0},//1
  {1,1,0,1,1,0,1},//2
  {1,1,1,1,0,0,1},//3
  {0,1,1,0,0,1,1},//4
  {1,0,1,1,0,1,1},//5
  {1,0,1,1,1,1,1},//6
  {1,1,1,0,0,0,0},//7
  {1,1,1,1,1,1,1},//8
  {1,1,1,1,0,1,1},//9
  {1,1,1,0,1,1,1},//A
  {0,0,1,1,1,1,1},//b
  {1,0,0,1,1,1,0},//C
  {0,1,1,1,1,0,1},//d 
  {1,0,0,1,1,1,1},//E 
  {1,0,0,0,1,1,1}//F
};

void setup(){

  for(int i=0;i<8;i++){
    pinMode(sw[i],INPUT);
  }

  for(int i=0;i<7;i++){
    pinMode(segPins[i],OUTPUT);
  }

  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);

}

void loop(){
  int high4 = (PIND >> 4) & 0x0F; 
  int low4 = PIND & 0x0F;
  int result = 0;
  if (high4 == low4) {
    result = low4;
    digitalWrite(10,HIGH); 
  }
  else if (high4 > low4){
    result = high4 - low4;
    digitalWrite(9,HIGH);
  }
  else{
    result = low4 - high4;
    digitalWrite(11,HIGH);
  }
  for (int i=0;i<7;i++){
    digitalWrite(segPins[i],num[result][i]);
  }
}
