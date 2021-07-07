// ===== TAGISANG ROBOTICS 2014
// ===== GRIDLOCK (GOAL KEEPER)
// ===== EDITED OCTOBER 10 2014
// by Calvin Ng
// algorithm by Calvin Ng and Arvy Ngo

// main changes from 2013:
// massive refactor
// compressed message length
// reduced latency

// initial coordinates
int x_origin = 515;
int y_origin = 515;

int x_max = 1023;
int x_min = 0;

int y_max = 1023;
int y_min = 0;

int y_coor = 515;
int x_coor = 501;

// setting of min and max power value
int max_power = 255;
int min_power = 0;

// mapping of pins is found in TRC training material
int back_sensorPin = A2; // X-AXIS Attached to the back sensor of Controller. Detects motion along x-axis (left and right). --> left joystick
int side_sensorPin = A1; // Y-AXIS Attached to the side sensor of Controller. Detects motion along y-axis (forward and backward). --> left joystick
int expand = A5;  // R1 button (used for triggering the expansion)
int fender = 3;   // L1 button (used for the fender)
int hook1 = 5;    // '1' button
int hook2 = 6;    // '2' button

int expand_status = 0;
int fender_status = 0;
int hook1_status = 0;
int hook2_status = 0;
int coded_status = 0;

String message1;
String message2;
String message;

float PowerLR = 0.0;
float PowerL = 0.0;
float PowerR = 0.0;

// ==== used in assign_power_to_motor() function only =========================================
int motor_forward_left = 12;
int motor_backward_left = 7;
int motor_forward_right = 8;
int motor_backward_right = 11;

int left_pwm = 9;
int right_pwm = 4;

int mot_pwr_left;
int mot_pwr_right;

int mot_dir_left;
int mot_dir_right;
// =============================================================================================


void setup(){
  
  analogReference(DEFAULT);
  
  Serial.begin(38400);
  
  // ==== used in assign_power_to_motor() function only ========================================= 
  pinMode(motor_backward_left, OUTPUT); 
  pinMode(motor_forward_left, OUTPUT);
  pinMode(motor_backward_right, OUTPUT);
  pinMode(motor_forward_right, OUTPUT);
  //pinMode(gather, OUTPUT);
  pinMode(left_pwm, OUTPUT);
  pinMode(right_pwm, OUTPUT);
  // ============================================================================================
}

float compute_initial_power(){
  
  y_coor = analogRead(side_sensorPin);
  //x_coor = analogRead(back_sensorPin);  
  
  //Serial.print("Y COOR:  ");
  //Serial.print(y_coor);
  //Serial.print("\n");
  //Serial.print("X COOR:  ");
  //Serial.print(x_coor);
  //Serial.print("\n");  
  
  float Init_PowerLR = 0;
  
  if(y_coor > y_origin){     // joystick pulled down
    Init_PowerLR = -1.0 * ((float(y_coor) - float(y_origin)) / (float(y_max) - float(y_origin))) * float(max_power); 
  }
  else if(y_coor < y_origin){ // joystick pushed up
    Init_PowerLR = ((float(y_origin) - float(y_coor)) / (float(y_origin) - float(y_min))) * float(max_power); 
  }
  
  PowerLR = Init_PowerLR;
  PowerL = Init_PowerLR; //* (0.85);
  PowerR = Init_PowerLR;
  
  //Serial.print("Initial Power:  ");
  //Serial.print(PowerLR);
  //Serial.print("\n");

  return Init_PowerLR;  

}

int get_coded_status(int expand, int fender, int hook1, int hook2){
  
  int coded_status;
  
  int hook = hook1 * hook2;  
  
  //       Expand      Fender      Hook       
  // 0 -     0           0          0
  // 1 -     0           0          1
  // 2 -     0           1          0
  // 3 -     0           1          1
  // 4 -     1           0          0
  // 5 -     1           0          1
  // 6 -     1           1          0
  // 7 -     1           1          1

  if(expand == 0 && fender == 0 && hook == 0){  
    coded_status = 0;  
  }
  else if(expand == 0 && fender == 0 && hook == 1){  
    coded_status = 1;  
  }
  else if(expand == 0 && fender == 1 && hook == 0){  
    coded_status = 2;  
  }
  else if(expand == 0 && fender == 1 && hook == 1){  
    coded_status = 3;  
  }
  else if(expand == 1 && fender == 0 && hook == 0){  
    coded_status = 4;  
  }
  else if(expand == 1 && fender == 0 && hook == 1){  
    coded_status = 5;  
  }
  else if(expand == 1 && fender == 1 && hook == 0){  
    coded_status = 6;  
  }
  else if(expand == 1 && fender == 1 && hook == 1){  
    coded_status = 7;  
  }   
  
  return coded_status;

} 

void compute_power_LR(){
  
  x_coor = analogRead(back_sensorPin);
  
  expand_status = digitalRead(expand);
  fender_status = digitalRead(fender);
  hook1_status = digitalRead(hook1);
  hook2_status = digitalRead(hook2);
  
  coded_status = get_coded_status(expand_status, fender_status, hook1_status, hook2_status);
    
  float bias_factor = 0.0;
  
  if(x_coor > (x_origin + 100)){ // || y_coor > (y_origin - 100)){   // biased to the right
    
    if(y_coor > (y_origin - 100) && y_coor < (y_origin + 100)){   // spinning
    
      if(x_coor > (x_origin + 100)){
        
        PowerL = -1.0 * ((float(x_origin) - float(x_coor)) / (float(x_origin) - float(x_min))) * float(max_power);
        PowerR = -1.0 * PowerL;
          
      }
    }
    else{  // turning
      float x_diff = float(x_coor) - float(x_max); 
      bias_factor = abs(x_diff) / (float(x_max) - float(x_origin)); 
      PowerR = PowerLR * bias_factor;
      //Serial.print("Bias Factor (LEFT):  ");
      //Serial.print(bias_factor);
      //Serial.print("\n");      
  
    }
  }
  else if(x_coor < (x_origin - 100)){ // || y_coor > (y_origin - 100)){   // biased to the left
  
    if(y_coor < (y_origin + 100) && y_coor > (y_origin - 100)){   // spinning
  
      if(x_coor < (x_origin - 100)){
        
        PowerR = -1.0 * ((float(x_coor) - float(x_origin)) / (float(x_max) - float(x_origin))) * float(max_power);
        PowerL = -1.0 * PowerR;
        
      }
    }
    else{  // turning
      float x_diff = float(x_min) - float(x_coor);
      bias_factor = abs(x_diff) / (float(x_origin) - float(x_min));   
      PowerL = PowerLR * bias_factor;
      //Serial.print("Bias Factor (LEFT):  ");
      //Serial.print(bias_factor);
      //Serial.print("\n");

    }
  }
  else if ( (x_coor > (x_origin - 100) && x_coor < (x_origin + 100) && y_coor > (y_origin - 100) && y_coor < (y_origin + 100) ) ){
    PowerL = 0.0;
    PowerR = 0.0;
  }

}


String get_message_to_send(){
  
  float PowerTS = 0; 
  float PowerRS = 0;
  
  if(PowerL > 0.0 && PowerL <= 255.0){   // positive left motor
     message1 = String(1000 + int(PowerL));         
  }
  else if(PowerL > 255.0){
    message1 = String(1000 + 255);
  }
  else if(PowerL == 0.0){   // neutral
     message1 = "0000";  //String(0 + int(PowerL));         
  }
  else if(PowerL < 0.0 && PowerL >= -255.0){   // negative
     message1 = String(2000 + (-1 * int(PowerL)) );
  }
  else if(PowerL < -255.0){
    message1 = String(2000 + 255);
  }
  
  // ====================== LEFT MOTOR NETURAL ==============================================================
  if(message1[0] == '0' && coded_status == 0){   // left motor neutral, expand = 0, fender = 0, hook = 0
    message1[0] = 'A';
  }
  else if(message1[0] == '0' && coded_status == 1){   // left motor neutral, expand = 0, fender = 0, hook = 1
    message1[0] = 'B';
  }
  else if(message1[0] == '0' && coded_status == 2){   // left motor neutral, expand = 0, fender = 1, hook = 0
    message1[0] = 'C';
  }
  else if(message1[0] == '0' && coded_status == 3){   // left motor neutral, expand = 0, fender = 1, hook = 1
    message1[0] = 'D';
  }
  else if(message1[0] == '0' && coded_status == 4){   // left motor neutral, expand = 1, fender = 0, hook = 0
    message1[0] = 'E';
  }
  else if(message1[0] == '0' && coded_status == 5){   // left motor neutral, expand = 1, fender = 0, hook = 1
    message1[0] = 'F';
  }
  else if(message1[0] == '0' && coded_status == 6){   // left motor neutral, expand = 1, fender = 1, hook = 0
    message1[0] = 'G';
  }
  else if(message1[0] == '0' && coded_status == 7){   // left motor neutral, expand = 1, fender = 1, hook = 1
    message1[0] = 'H';
  }   
  // =========================================================================================================
  
  
  // ====================== LEFT MOTOR FORWARD ===============================================================
  if(message1[0] == '1' && coded_status == 0){   // left motor forward, expand = 0, fender = 0, hook = 0
    message1[0] = 'I';
  }
  else if(message1[0] == '1' && coded_status == 1){   // left motor forward, expand = 0, fender = 0, hook = 1
    message1[0] = 'J';
  }
  else if(message1[0] == '1' && coded_status == 2){   // left motor forward, expand = 0, fender = 1, hook = 0
    message1[0] = 'K';
  }
  else if(message1[0] == '1' && coded_status == 3){   // left motor forward, expand = 0, fender = 1, hook = 1
    message1[0] = 'L';
  }
  else if(message1[0] == '1' && coded_status == 4){   // left motor forward, expand = 1, fender = 0, hook = 0
    message1[0] = 'M';
  }
  else if(message1[0] == '1' && coded_status == 5){   // left motor forward, expand = 1, fender = 0, hook = 1
    message1[0] = 'N';
  }
  else if(message1[0] == '1' && coded_status == 6){   // left motor forward, expand = 1, fender = 1, hook = 0
    message1[0] = 'O';
  }
  else if(message1[0] == '1' && coded_status == 7){   // left motor forward, expand = 1, fender = 1, hook = 1
    message1[0] = 'P';
  }   
  // =========================================================================================================
  
  // ====================== LEFT MOTOR BACKWARD ===============================================================
  if(message1[0] == '2' && coded_status == 0){   // left motor backward, expand = 0, fender = 0, hook = 0
    message1[0] = 'Q';
  }
  else if(message1[0] == '2' && coded_status == 1){   // left motor backward, expand = 0, fender = 0, hook = 1
    message1[0] = 'R';
  }
  else if(message1[0] == '2' && coded_status == 2){   // left motor backward, expand = 0, fender = 1, hook = 0
    message1[0] = 'S';
  }
  else if(message1[0] == '2' && coded_status == 3){   // left motor backward, expand = 0, fender = 1, hook = 1
    message1[0] = 'T';
  }
  else if(message1[0] == '2' && coded_status == 4){   // left motor backward, expand = 1, fender = 0, hook = 0
    message1[0] = 'U';
  }
  else if(message1[0] == '2' && coded_status == 5){   // left motor backward, expand = 1, fender = 0, hook = 1
    message1[0] = 'V';
  }
  else if(message1[0] == '2' && coded_status == 6){   // left motor backward, expand = 1, fender = 1, hook = 0
    message1[0] = 'W';
  }
  else if(message1[0] == '2' && coded_status == 7){   // left motor backward, expand = 1, fender = 1, hook = 1
    message1[0] = 'X';
  }   
  // =========================================================================================================
   
  //Serial.print("Message1:  ");
  //Serial.print(message1);
  //Serial.print("\n");
  
  if(PowerR > 0.0 && PowerR <= 255.0){   // positive left motor
     message2 = String(1000 + int(PowerR));        
  }
  else if(PowerR > 255.0){
     message2 = String(1000 + 255);
  }
  else if(PowerR == 0.0){   // neutral
     message2 = "0000"; //String(0 + (-1 * int(PowerR)) );
  }
  else if(PowerR < 0.0 && PowerR >= -255.0){  // negative
     message2 = String(2000 + (-1 * int(PowerR)) );
  }
  else if(PowerR < -255.0){
     message2 = String(2000 + 255);
  }
  
  //Serial.println(fender_status);
  
  if(message2[0] == '0'){   // right motor neutral
    //Serial.print("GOT A");
    message2[0] = 'Y';
  }
  else if(message2[0] == '1'){   // right motor forward
    message2[0] = 'Z';
  }
  else if(message2[0] == '2'){   // right motor reverse
    message2[0] = '!';
  }  
  
  //Serial.print("Message2:  ");
  //Serial.print(message2);
  //Serial.print("\n");
  
  //message = (int(message1.toCharArray()) * 10000) + int(message2.toCharArray());
  message = message1 + message2; // + String(shoot_status);  
  
  return message;

}

void loop(){
  
  float Init_Power = compute_initial_power();
  //Serial.print("Initial Power:  ");
  //Serial.print(Init_Power);
  //Serial.print("\n");
  
  compute_power_LR();
  
  //Serial.print("Power Left:  ");
  //Serial.print(PowerL);
  //Serial.print("\n");
  //Serial.print("Power Right:  ");
  //Serial.print(PowerR);
  //Serial.print("\n");
  
  String message = get_message_to_send();
  //Serial.print("Power To Send:  ");
  Serial.print(message);
  //delay(2000);
  //Serial.print("\n");
  //Serial.print(shoot_status);
  //Serial.print("\n\n");
  //assign_power_to_motor();    
  
  delay(100);  
}


//===============================================================================================================
void assign_power_to_motor(){
  
   // initialize motor power level to zero
   analogWrite (left_pwm, 0);
   analogWrite (right_pwm, 0);
   
   // initialize motor direction to all low
   digitalWrite (motor_backward_left, LOW);
   digitalWrite (motor_forward_left, LOW);
   digitalWrite (motor_backward_right, LOW);
   digitalWrite (motor_forward_right, LOW);
   
   int PowerL_to_send = int(PowerL); 
   int PowerR_to_send = int(PowerR);
  
   mot_pwr_left = abs(PowerL_to_send);    // left motor power level
   mot_pwr_right = abs(PowerR_to_send);   // right motor power level
                                                  
   if (message1[0] == '1') {    // if left motor forward
     digitalWrite (motor_backward_left, LOW);
     digitalWrite (motor_forward_left, HIGH); 
     analogWrite (left_pwm, mot_pwr_left);
     
     
     Serial.print("left forward");
     Serial.print("\n");
     Serial.print("Left Power Level: ");
     Serial.print(mot_pwr_left); 
     Serial.print("\n");
     
   }
   else if (message1[0] == '2')  {   // if left motor reversed
     digitalWrite (motor_backward_left, HIGH);
     digitalWrite (motor_forward_left, LOW);
     analogWrite (left_pwm, mot_pwr_left);
     
     
     Serial.print("left backward");
     Serial.print("\n");
     Serial.print("Left Power Level: ");
     Serial.print(mot_pwr_left);
     Serial.print("\n"); 
       
   }
   else {
     digitalWrite (motor_backward_left, LOW);
     digitalWrite (motor_forward_left, LOW); 
     
     
     Serial.print("left nothing");
     Serial.print("\n");
     Serial.print("Left Power Level: ");
     Serial.print(mot_pwr_left);
     Serial.print("\n");
     
   }

   if (message2[0] == '1') {      // if right motor forward
     digitalWrite (motor_backward_right, LOW);
     digitalWrite (motor_forward_right, HIGH); 
     analogWrite (right_pwm, mot_pwr_right);
     
     
     Serial.print("right forward");
     Serial.print("\n");
     Serial.print("Right Power Level: ");
     Serial.print(mot_pwr_right);      
     Serial.print("\n");
     
   }
   else if (message2[0] == '2') {   // if right motor backward
     digitalWrite (motor_backward_right, HIGH);
     digitalWrite (motor_forward_right, LOW);
     analogWrite (right_pwm, mot_pwr_right);
     
     
     Serial.print("right backward");
     Serial.print("\n");
     Serial.print("Right Power Level: ");
     Serial.print(mot_pwr_right);      
     Serial.print("\n");
     
   }
   else {    
     digitalWrite (motor_backward_right, LOW);
     digitalWrite (motor_forward_right, LOW);
     
     
     Serial.print("right neutral"); 
     Serial.print("\n");
     Serial.print("Right Power Level: ");
     Serial.print(mot_pwr_right);      
     Serial.print("\n");
     
   }     
     
   Serial.print("\n");  
   //delay(2000);

}


/*
void compute_power_LR(){
  
  x_coor = analogRead(back_sensorPin);
  
  shooter_status = digitalRead(shooter);
  
  if(shooter_status == 1){
     gather_status = 1;  
  }
  else{  
     gather_status = digitalRead(gather);
  }  
  
  float bias_factor = 0.0;
  
  if(x_coor > (x_origin)){ // || y_coor > (y_origin - 100)){   // biased to the right
    
    if(y_coor > (y_origin) && y_coor < (y_origin)){   // spinning
    
      if(x_coor > (x_origin)){
        
        PowerL = -1.0 * ((float(x_origin) - float(x_coor)) / (float(x_origin) - float(x_min))) * float(max_power);
        PowerR = -1.0 * PowerL;
          
      }
    }
    else{  // turning
      float x_diff = float(x_coor) - float(x_max); 
      bias_factor = abs(x_diff) / (float(x_max) - float(x_origin)); 
      PowerR = PowerLR * bias_factor;
      //Serial.print("Bias Factor (LEFT):  ");
      //Serial.print(bias_factor);
      //Serial.print("\n");      
  
    }
  }
  else if(x_coor < (x_origin)){ // || y_coor > (y_origin - 100)){   // biased to the left
  
    if(y_coor < (y_origin) && y_coor > (y_origin)){   // spinning
  
      if(x_coor < (x_origin)){
        
        PowerR = -1.0 * ((float(x_coor) - float(x_origin)) / (float(x_max) - float(x_origin))) * float(max_power);
        PowerL = -1.0 * PowerR;
        
      }
    }
    else{  // turning
      float x_diff = float(x_min) - float(x_coor);
      bias_factor = abs(x_diff) / (float(x_origin) - float(x_min));   
      PowerL = PowerLR * bias_factor;
      //Serial.print("Bias Factor (LEFT):  ");
      //Serial.print(bias_factor);
      //Serial.print("\n");

    }
  }
  else if ( (x_coor > (x_origin) && x_coor < (x_origin) && y_coor > (y_origin) && y_coor < (y_origin) ) ){
    PowerL = 0.0;
    PowerR = 0.0;
  }

}
*/


