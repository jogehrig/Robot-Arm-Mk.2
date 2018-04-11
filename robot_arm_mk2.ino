#include <Servo.h>
#include <Azande.h>//DEBUG




define_int_event(eventPositionPoti, "Position", 6,"Poti",,,);//DEBUG

Azande azande(Serial);//DEBUG




Servo servo_0; //NULL
Servo servo_1; //Basis
Servo servo_2; //Schulter
Servo servo_3; //Ellbogen
Servo servo_4; //Handgelenk
Servo servo_5; //Zange

int sensorPin0 = A0; //NULL
int sensorPin1 = A1; //Basis
int sensorPin2 = A2; //Schulter
int sensorPin3 = A3; //Ellbogen
int sensorPin4 = A4; //Handgelenk
int sensorPin5 = A5; //Zange

unsigned int  verz = 0;
int count0, arrayStep, arrayMax, countverz, Taster, stepsMax, steps, time = 1000, del = 1000, temp;
//arraystep = position in array
//arrayMax = max steps safed to array
//countverz = delay between moves
//stepsMax = longest way a servo has to travel
//steps = single steps for a move between stored positions

long previousMillis1 = 0;
long previousMillis2 = 0;
long previousMillis3 = 0;
long previousMillis4 = 0;
long previousMicros = 0;
unsigned long currentMillis = millis();
unsigned long currentMicros = micros();

int Delay[7] = {0,0,1,1,1,1,1}; // array to map gripper pot to delay in seconds
int SensVal[6];
float dif[6], ist[6], soll[6],  dir[6]; // difference between stored and momentary position
int joint0[180];//180 Positionen nacheinander
int joint1[180];
int joint2[180];
int joint3[180];
int joint4[180];
int joint5[180];

int top = 179;
boolean playmode = false, Step = false;

void setup()
{
  pinMode(4, INPUT);
  pinMode(2, INPUT);
  pinMode(13, OUTPUT);//LED
  digitalWrite(13, HIGH);//LED on
  servo_0.attach(3);
  servo_0.attach(5);
  servo_0.attach(6);
  servo_0.attach(9);
  servo_0.attach(10);
  servo_0.attach(11);
  Serial.begin(9600); //DEBUG
  Serial.println("Ready");     
  delay(1000);
  digitalWrite(13, LOW);
  define_int_event(eventPositionPoti, "Position", 6,"Poti",,,);//DEBUG
Azande azande(Serial);//DEBUG


}

void loop()
{
  currentMillis = millis();
  currentMicros = micros();
  
  Button();
  
  if(!playmode) //Manuell
  {        
    if(currentMillis - previousMillis1 > 25) // 25ms till next mode update
    {
      if (arrayStep < top) 
      {
        previousMillis1 = currentMillis; //reset
        readPot();
        mapping();
        move_servo();
        
        //record();   
      } //counter < max
    } //step check
  } //manuell
   
  else if(playmode) // play
  {
    if (Step) // next step from array
    {
      digitalWrite(13, HIGH); //LED
      if (arrayStep < arrayMax)
      {
        arrayStep += 1;
        Read();
        calculate(); //find biggest travel distance and calculate the other servos (they have to do smaller steps to be finished at same time)
        Step = 0;
        digitalWrite(13, LOW);  
      }
      else // array read finished > start over
      {
        arrayStep = 0;
        calc_pause(); //delay between moves read from potentiometer
        countverz = 0; //used for the delay
        while(countverz < verz) //verz = time got from calc_pause();
        { countverz += 1;
          calc_pause();
          digitalWrite(13, HIGH); delay(25);   
          digitalWrite(13, LOW); delay(975); 
        }
      }
      //Serial.println(arrayStep);
    }
    else
    {
      if (currentMicros - previousMicros > time)
      { previousMicros = currentMicros;
        play_servo(); 
      }
    }
  }// ende playmode

    while (digitalRead(4) == false)
      { 
        digitalWrite(13, HIGH); delay(500);   
        digitalWrite(13, LOW); delay(500);
      }

}



void calc_pause() //pause between pot. movements
{
    readPot();
    temp = SensVal[0];
    if (temp < 0) temp = 0;
    temp = map(temp, 0, 680, 0 ,5); 
    verz = Delay[temp]; //delay in second
    Serial.print(temp);
          Serial.print(" ");
          Serial.print(verz);
          Serial.print(" ");
          Serial.println(countverz);
}

void readPot()
{
   SensVal[0] = analogRead(sensorPin0);
   SensVal[1] = analogRead(sensorPin1);
   SensVal[2] = analogRead(sensorPin2);
   SensVal[3] = analogRead(sensorPin3);
   SensVal[4] = analogRead(sensorPin4);
   SensVal[5] = analogRead(sensorPin5);
   azande.send(eventPositionPoti,SensVal[0]);//DEBUG

  //Serial.print(SensVal[0]);Serial.print(" "); //DEBUG
}
void mapping()
{
  ist[0] = map(SensVal[0], 0, 1023, 1000, 2000);//SERVOS NOCH ZU KALIBRIEREN
  ist[1] = map(SensVal[1], 0, 1023, 1000, 2000);
  ist[2] = map(SensVal[2], 550, 1023, 1000, 2000);
  ist[3] = map(SensVal[3], 300, 770, 1000, 2000);
  ist[4] = map(SensVal[4], 0, 1023, 1000, 2000);
  ist[5] = map(SensVal[5], 0, 1023, 1000, 2000);

 //Serial.println(ist[0]); //DEBUG
}
void record()
{
    joint0[arrayStep] = ist[0];
    joint1[arrayStep] = ist[1];
    joint2[arrayStep] = ist[2];
    joint3[arrayStep] = ist[3];
    joint4[arrayStep] = ist[4];
    joint5[arrayStep] = ist[5];

}
void Read()
{
    soll[0] = joint0[arrayStep];
    soll[1] = joint1[arrayStep];
    soll[2] = joint2[arrayStep];
    soll[3] = joint3[arrayStep];
    soll[4] = joint4[arrayStep];
    soll[5] = joint5[arrayStep];
}
void move_servo()
{       
  servo_0.writeMicroseconds(ist[0]);
  servo_1.writeMicroseconds(ist[1]);
  servo_2.writeMicroseconds(ist[2]);
  servo_3.writeMicroseconds(ist[3]);
  servo_4.writeMicroseconds(ist[4]);
  servo_5.writeMicroseconds(ist[5]);

}
void calculate()
{
      dif[0] = abs(ist[0]-soll[0]);
      dif[1] = abs(ist[1]-soll[1]);
      dif[2] = abs(ist[2]-soll[2]);
      dif[3] = abs(ist[3]-soll[3]);
      dif[4] = abs(ist[4]-soll[4]);
      dif[5] = abs(ist[5]-soll[5]);

      stepsMax = max(dif[0],dif[1]);// biggest travel way from all servos
      stepsMax = max(stepsMax,dif[2]);
      stepsMax = max(stepsMax,dif[3]);
      stepsMax = max(stepsMax,dif[4]);
      stepsMax = max(stepsMax,dif[5]);
      
      //Serial.println(stepsMax); //DEBUG
      
      if (stepsMax < 500)
        del = 1200;
      else
        del = 600;
      
     // calculating single step for each servo to move all servos in a loop (stepsMax times done) with different values
      if (soll[0] < ist[0]) dir[0] = 0-dif[0]/stepsMax; else dir[0] = dif[0]/stepsMax;
      if (soll[1] < ist[1]) dir[1] = 0-dif[1]/stepsMax; else dir[1] = dif[1]/stepsMax;
      if (soll[2] < ist[2]) dir[2] = 0-dif[2]/stepsMax; else dir[2] = dif[2]/stepsMax;
      if (soll[3] < ist[3]) dir[3] = 0-dif[3]/stepsMax; else dir[3] = dif[3]/stepsMax;
      if (soll[4] < ist[4]) dir[4] = 0-dif[4]/stepsMax; else dir[4] = dif[4]/stepsMax;
      if (soll[5] < ist[5]) dir[5] = 0-dif[5]/stepsMax; else dir[5] = dif[5]/stepsMax;

}
void play_servo()
{
    steps += 1;
    if (steps < stepsMax)
    {
      if(steps == 20) time = del*4;         // ramp up 
      else if(steps == 40) time = del*3;    // delay in ms we wait in the mainloop until
      else if(steps == 80) time = del*2;    // a micro step will be done
      else if(steps == 100) time = del-1;
      
      if(steps == stepsMax-200) time = del*2;// stop -ramp down (200 microsteps before end time will be increased
      else if(steps == stepsMax-80) time = del*3;
      else if(steps == stepsMax-40) time = del*4;
      else if(steps == stepsMax-20) time = del*5;
      
      ist[0] += dir[0]; // set new pos

      servo_0.writeMicroseconds(ist[0]);
      servo_1.writeMicroseconds(ist[1]);
      servo_2.writeMicroseconds(ist[2]);
      servo_3.writeMicroseconds(ist[3]);
      servo_4.writeMicroseconds(ist[4]);
      servo_5.writeMicroseconds(ist[5]);
    }
    else
    {
      Step = 1; // next step aus array lesen
      steps = 0; // servo zwischenschritte
    }
}

void data_out()
{
  int i = 0;
  while(i < arrayMax)
  {
    digitalWrite(13, HIGH);
    i += 1;
    Serial.print(joint0[i]); Serial.print(", ");
  }
  Serial.println("Joint0");
  i = 0;

}

void Button() // check buttons for single/ doubleclick
{
  if (digitalRead(2) == false)
  {
    delay(1);
    if (digitalRead(2) == true) // taster losgelassen
    {
      if (Taster == 0)
      {
        Taster = 1;
        previousMillis3 = currentMillis;
        Serial.print("Record "); Serial.println(Taster); 
      }
      else if ((Taster == 1) && (currentMillis - previousMillis3 < 250))
      {
        Taster = 2;
        Serial.println(Taster); 
      }
      /*else if ((Taster == 2) && (currentMillis - previousMillis3 < 500))
      {
        Taster = 3;
        Serial.println(Taster); 
      }*/
    }
  }
    
    if ((Taster == 1) && (currentMillis - previousMillis3 > 1000)) // write to array
    {
      arrayStep += 1;
      arrayMax = arrayStep;
      record();
      Taster = 0;
      playmode = false;
      Serial.print("Record Step: "); Serial.println(arrayStep);
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
    }
    else if (Taster == 2)
    {
      arrayStep = 0;
      playmode = true;
      Taster = 0;
      Step = 1;
      Serial.println("playmode ");
      data_out();
      delay(250);   
      digitalWrite(13, LOW);    
    }

    if (currentMillis - previousMillis3 > 2000)
    {
      Taster = 0;
      Serial.println("restart ");
    }
}
