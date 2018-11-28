// Motor A
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;

int speedLeftPin = 10;
int speedRightPin = 11;

int leftSensorPin = A0;
int rightSensorPin = A1;

int leftSensorValue;
int rightSensorValue;
int difValue;

int buttonState = 2;
int buttonStateValue;

double P;
int setpoint=420;
int hist = 0;
void AdelanteDer ()
{
 //Direccion motor A
 digitalWrite (IN1, HIGH);
 digitalWrite (IN2, LOW);

}

void AtrasDer ()
{
 //Direccion motor A
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, HIGH);
}

void PararDer ()
{
 //Direccion motor A
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, LOW);
 
}

void AdelanteIzq ()
{
 //Direccion motor A
 digitalWrite (IN3, HIGH);
 digitalWrite (IN4, LOW);

}

void AtrasIzq ()
{
 //Direccion motor A
 digitalWrite (IN3, LOW);
 digitalWrite (IN4, HIGH);
}

void PararIzq ()
{
 //Direccion motor A
 digitalWrite (IN3, LOW);
 digitalWrite (IN4, LOW);
 
}

void probarRuedas(){
  
AdelanteDer();
delay(1000);

PararDer();
AtrasDer();
delay(1000);

PararDer();
delay(1000);

AdelanteIzq();
delay(1000);

PararIzq();
AtrasIzq();
delay(1000);

PararIzq();
delay(1000);
}

void Avanzar(){
AdelanteIzq();
AdelanteDer();
}

void Retroceder(){
AtrasIzq();
AtrasDer();
}

void Parar(){
PararIzq();
PararDer();
}

void Vuelta(){
  AdelanteIzq();
  AtrasDer();
  delay(300);
  Parar();
}
void SeguirLinea(){
 //line detected by both
  if(analogRead(leftSensorPin)<=400 && analogRead(rightSensorPin)<=400){
    //stop
    Parar();
  }
  //line detected by left sensor
  else if(analogRead(leftSensorPin)<=400 && !analogRead(rightSensorPin)<=400){
    //turn left
    AdelanteDer();
    PararIzq();
  }
  //line detected by right sensor
  else if(!analogRead(leftSensorPin)<=400 && analogRead(rightSensorPin)<=400){
    //turn right
    AdelanteIzq();
    PararDer();
}

}

void DistanciaParedRight(){
  rightSensorValue = analogRead(rightSensorPin);
    if(rightSensorValue<setpoint-hist){
      AdelanteDer();
    }
  else if(rightSensorValue>setpoint+hist){
      AtrasDer();
    } 
}

void DistanciaParedLeft(){
  leftSensorValue = analogRead(leftSensorPin);
    if(leftSensorValue<setpoint-hist){
      AdelanteIzq();
    }
  else if(leftSensorValue>setpoint+hist){
      AtrasIzq();
    } 
}

void Checkvuelta(){
  leftSensorValue = analogRead(leftSensorPin);
  rightSensorValue = analogRead(rightSensorPin);
  if (leftSensorValue < setpoint + 10 && leftSensorValue > setpoint -10 && rightSensorValue < setpoint +10 && rightSensorValue > setpoint -10){
    //Vuelta();
  }
}

void setup() {
  // put your setup code here, to run once:
 //Serial.begin(9600);
 pinMode (buttonState, INPUT);
 
 pinMode (IN1, OUTPUT);
 pinMode (IN2, OUTPUT);
 pinMode (IN3, OUTPUT);
 pinMode (IN4, OUTPUT);

 pinMode (speedLeftPin, OUTPUT);
 pinMode (speedRightPin, OUTPUT);

 

analogWrite(speedLeftPin, 180
);
analogWrite(speedRightPin, 150);


}

void loop() {
  // put your main code here, to run repeatedly:
DistanciaParedLeft();
DistanciaParedRight();
Checkvuelta();
//buttonStateValue = digitalRead(buttonState);
//if (buttonStateValue != LOW) {
 // while(1>0){
  //DistanciaPared();  
  //}
    
//}


//leftSensorValue = analogRead(leftSensorPin);
//rightSensorValue = analogRead(rightSensorPin);

//Serial.print(rightSensorValue);
//Serial.print("\n");
//delay(100);


}
