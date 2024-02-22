
int potR = A1;
int potL =A0;

int motorR = 5;
int motorL =3;

int slideR= 4;
int slideL = 2;

int motorRstate = 0;
int motorLstate = 0;


char ser;
void setup()
{ Serial.begin(9600);
 
 pinMode(slideR,INPUT);
  pinMode(slideL,INPUT);
 
}

void loop()
{if(Serial.available()>1)
{ ser = Serial.read();
  switch(ser)
  {case 'L':
     motorLstate = 1;
  break;
  case 'R':
    motorRstate = 1;
  break;
  case 'S':
    motorRstate = 0;
    motorLstate = 0;
    break;}}
 
 if(digitalRead(slideR)==1 || motorRstate)
 {analogWrite(motorR,analogRead(potR)/4);}
 else
 {analogWrite(motorR,0);}

if(digitalRead(slideL)==1 || motorLstate)
 {analogWrite(motorL,analogRead(potL)/4);}
 else
 {analogWrite(motorL,0);}

}
 


 
  

  
