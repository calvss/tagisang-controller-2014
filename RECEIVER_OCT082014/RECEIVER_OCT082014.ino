// RECEIVER code last modified OCT 08 2014
// by Calvin Ng
// algorithm by Calvin Ng and Arvy Ngo

#include <SoftwareSerial.h>

SoftwareSerial mySerial(12, 13);

int direction_left = 3;
int direction_right = 7;

int pwm_left = 11;
int pwm_right = 5;

char motor_status_right = 'N'; // for display in serial monitor only
char motor_status_left = 'N'; // for display in serial monitor only

int gather = A0; // gather pin
int shooter = A5; // shooter pin


int mechanisism_ON = 55; // power level is set to maximum 
int mechanisism_OFF = 255; // power level is set to 0

int leftm_direction;
int rightm_direction;
int leftm_power;
int rightm_power;

int leftm_3rd_digit;
int leftm_2nd_digit;
int leftm_1st_digit;

int rightm_3rd_digit;
int rightm_2nd_digit;
int rightm_1st_digit;

int shooter_status = 0;
int gather_status = 0;

void setup(){
  
   //Serial.begin (38400);
   mySerial.begin (38400);
   pinMode (direction_left , OUTPUT);
   pinMode (direction_right , OUTPUT);
   pinMode (gather , OUTPUT); 
   pinMode (shooter, OUTPUT);
   
   pinMode (10, OUTPUT);
    
   // grounds the following pins
   pinMode (6, OUTPUT);
   digitalWrite(6, LOW);
   pinMode (2, OUTPUT);
   digitalWrite(2, LOW);
   pinMode (4, OUTPUT);
   digitalWrite(4, LOW);
   pinMode (8, OUTPUT);
   digitalWrite(8, LOW);
}


void assign_to_motors(){
  

    // ============ right motor =========================================================
    if(rightm_direction == 'I' || rightm_direction == 'J'){  // if right motor is forward      
      digitalWrite(direction_right, HIGH);
      
      motor_status_right = 'F'; // for display in serial monitor only
         
      if(rightm_power > 255){
        rightm_power = 255;  
      }
      
      analogWrite(pwm_right, rightm_power);
    }
    else if(rightm_direction == 'K' || rightm_direction == 'L'){  // if right motor is reverse
      
      digitalWrite(direction_right, LOW);     
      motor_status_right = 'B'; // for display in serial monitor only
    
      if(rightm_power > 255){
        rightm_power = 255;  
      }       
    
      analogWrite(pwm_right, rightm_power);
        
    }

    // ============ left motor =========================================================      
    if(leftm_direction == 'C' || leftm_direction == 'D'){   // if left motor is forward 
       digitalWrite(direction_left, HIGH);               
       motor_status_left = 'F'; // for display in serial monitor only
      
       if(leftm_power > 255){
         leftm_power = 255;  
       }      
       
       analogWrite(pwm_left, leftm_power);
    }     
    else if(leftm_direction == 'E' || leftm_direction == 'F'){   // if left motor is reverse
       digitalWrite(direction_left, LOW);      
       motor_status_left = 'B'; // for display in serial monitor only        
         
       if(leftm_power > 255){
         leftm_power = 255;  
       }       
       
       analogWrite(pwm_left, leftm_power);
    }
       
    // setting of power level
    if(rightm_direction == 'G' || rightm_direction == 'H' || leftm_direction == 'A' || leftm_direction == 'B'){   // neutral
       
       motor_status_left = 'N'; // for display in serial monitor only
       motor_status_right = 'N'; // for display in serial monitor only
       
       analogWrite(pwm_right, 0);
       analogWrite(pwm_left, 0);    
    }  
    else{   //  for backward and/or forward        
       
    }
    
    
    //===========shooter & gather statuses================================================
    
    if(leftm_direction == 'A' || leftm_direction == 'C' || leftm_direction == 'E'){   // set shooter to high
       digitalWrite(shooter, HIGH);
       digitalWrite(gather, HIGH);
       //analogWrite(mechanisism_switch, mechanisism_ON);
    }
      
    else if(rightm_direction == 'G' || rightm_direction == 'I' || rightm_direction == 'K'){   // set gather to high
       digitalWrite(gather, HIGH);
       //analogWrite(mechanisism_switch, mechanisism_ON);        
    }
    else{
       digitalWrite(shooter, LOW);
       digitalWrite(gather, LOW);
       //analogWrite(mechanisism_switch, mechanisism_OFF);           
    }

            
    //delay(100);
}


void loop() {
        
       int msg_ctr = 0;
       while(mySerial.available()){
         byte a = mySerial.read();
     
         if(msg_ctr == 0){
           leftm_direction = a;
           digitalWrite(10, HIGH);
           delay(10);
           digitalWrite(10, LOW);
           delay(10);       
         }
         else if(msg_ctr == 1){
           leftm_3rd_digit = a;
           digitalWrite(10, HIGH);
           delay(10);
           digitalWrite(10, LOW);
           delay(10); 
         }
         else if(msg_ctr == 2){
           leftm_2nd_digit = a;
           digitalWrite(10, HIGH);
           delay(10);
           digitalWrite(10, LOW);
           delay(10);
         }
         else if(msg_ctr == 3){
           leftm_1st_digit = a;
           digitalWrite(10, HIGH);
           delay(10);
           digitalWrite(10, LOW);
           delay(10);
         }
         else if(msg_ctr == 4){
           rightm_direction = a;
           digitalWrite(10, HIGH);
           delay(10);
           digitalWrite(10, LOW);
           delay(10);
         }
         else if(msg_ctr == 5){
           rightm_3rd_digit = a;
           digitalWrite(10, HIGH);
           delay(10);
           digitalWrite(10, LOW);
           delay(10);
         }
         else if(msg_ctr == 6){
           rightm_2nd_digit = a;
           digitalWrite(10, HIGH);
           delay(10);
           digitalWrite(10, LOW);
           delay(10);
         }
         else if(msg_ctr == 7){
           rightm_1st_digit = a;
           digitalWrite(10, HIGH);
           delay(10);
           digitalWrite(10, LOW);
           delay(10);
       
         }       
         msg_ctr++;    
       }
     
      
      
   leftm_power = abs((((leftm_3rd_digit - 48) * 100) + ((leftm_2nd_digit - 48) * 10) + (leftm_1st_digit - 48))); 
   rightm_power = abs((((rightm_3rd_digit - 48) * 100) + ((rightm_2nd_digit - 48) * 10) + (rightm_1st_digit - 48))); 
      
      
   assign_to_motors();
   
   
   /*
   Serial.print( "left motor direction: ");
   Serial.println( motor_status_left);
   Serial.print( "left motor power: ");
   Serial.println ( leftm_power );
   Serial.print( "right motor direction: ");
   Serial.println( motor_status_right); 
   Serial.print( "right motor power: ");
   Serial.println ( rightm_power );
   Serial.println ("");
   
   delay (800);
   */
   
   
} 






