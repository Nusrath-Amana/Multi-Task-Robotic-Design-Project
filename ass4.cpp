#include <webots/Robot.hpp>
//#include <webots/sensor.hpp>.........................
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <dos.h>
#include <webots/ImageRef.hpp>

//#include <webots/Keyboard.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include<cmath>

#include <fstream>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define baseSpeed 6

#define color_var 1 
                   //red-0
                   //blue-1
using namespace webots;
using namespace std;

bool break_main_loop;


int IR_THRESHOLD;
double set = 3500;
double sensorValues[10];
string IR_panel;
double line_le=0;
double kp=0.008;
double kd=0.0006;

//wall following
double wall_le=0;
int target=1560;
int following_distance = 1000;
double wall_kp=0.04;
double wall_kd=0.006;

//chess board


//defining variables 
double topsensorvalue;
double bottomsensorvalue;


// to check king is detected or not
bool king_found = false;



//chess squres IR colour  (white - 340, brown- 520)
//-------------------------------------
//for counting
double previous_ir = 520.00;

double new_ir ;
//---------------------------------------
//couting squres

int count =0;

//flages for forward and backword

int front_dir=0;  // 0 --> ffront


//flags for straight and right
int side_dir=0 ;// 1 --> backward


// staging ///
int substage=0;    // 

// substage 2 ---> after king found and put the box 

//varibles for couting squres









char linearNames[2][15] = { "left_arm", "right_arm"};
char servoName[12] = "main_Slider";



double front_;



// define the cuurent position of the hand
int current_hand_position = 0;








int timeStep;
//state variable
int state = 0;
int substate =0;

Motor *servo;
//Sensor *ds[3];
Motor *linear[2];

char motorNames[2][15] = { "left motor", "right motor" };
double UltraSonics[3];

double leftValue;
double rightValue;
string rotate;


bool error_found = 0;
int error_count = 0;
//char IR_panel[10];

////////////////////////////Objects



Motor *motors[5];




Robot *robot;
PositionSensor *ps_right;
PositionSensor *ps_left;
double mleft;
double mright;

//control motor speed
double Mdriver(double speed){

 if (speed > 0){
   if (speed > baseSpeed){
     speed = baseSpeed;
   }
 }else{
   if (speed < -baseSpeed){
     speed = -baseSpeed;
   }
 }
 
 return speed;
}

void forward()
    {motors[0]->setVelocity(MAX_SPEED);
     motors[1]->setVelocity(MAX_SPEED);
     } 

void stop()
{
  motors[0]->setVelocity(0);
  motors[1]->setVelocity(0);
}



void rotateRobot(Robot *robot,Motor **motors, double angle) {
  
  double wheelbase = 0.369; // Wheelbase of the robot in meters
  double wheelRadius = 0.04; // Radius of the wheels in meters
  // Set the speed of both motors to rotate the robot in place
  motors[0]->setVelocity(2.0);
  motors[1]->setVelocity(-2.0);

  // Calculate the duration of the rotation based on the angle to rotate
  double rotationDistance = wheelbase * angle;
  double rotationTime = rotationDistance / (wheelRadius * 2.0 * M_PI) * 1000.0; // In milliseconds

  // Wait for the robot to rotate by the specified angle
  robot->step(rotationTime);

  // Stop the motors
  motors[0]->setVelocity(0);
  motors[1]->setVelocity(0);
}



//----function to read the values of the sensors and convert to binary-------
string read(int ir_threshold=550){
 string IR_panel_val="";
 for (int i = 0; i < 8; i++){
   std::cout<<sensorValues[i]<<" ";
   if (sensorValues[i] > ir_threshold){
   
     sensorValues[i] = 1;
     IR_panel_val +="1";
   }else{
     sensorValues[i] = 0;
     IR_panel_val +="0";
   }
 }
 std::cout<<std::endl;
 return IR_panel_val; 
}
//----------------------end of the read() function----------------------------


//------------------function for PID calculation-------------------------------
//only PD needed

double PID_calc_linefollow()
 
          {
          double average = 0;
         double sum = 0;
         for (int i = 0; i < 8 ; i++){ 
           average += sensorValues[i] * i * 1000;
           sum += sensorValues[i];
           
         }
         
         double position = average / sum;  //---------weighted mean---------------------
         
         //double kp = 0.006;
         //double kd = 0.0008;
         //double ki = 0.0;
         double e = position - set; //error

        /*int panelWeights[8] = {-400,-300,-200,-100,100,200,300,400};
        double position = 0;
        
        int count1 = 0;

        for (int i = 0; i < 8; i++)
        {
            if (sensorValues[i]  == 1) // For White
            {
                position += panelWeights[i];
                count++;
            } 
        }
        
        //checking for zero division
        if (count1 == 0) 
        count1 = 1;
        
          
       double e = position /(count1 * 10); //error*/
       
       double p = kp * e;
       double d = kd * (e - line_le);
       //double i = ki * (e + le);
       double offset = p + d;
       line_le = e;  // le= last error
       return offset;
}
//--------------------end of the PID_calc() function---------------------------

double PID_calc_wallfollowing(double UV_read ){
            
              double e= (target - UV_read);
              double p = wall_kp * e;
              double d = wall_kd * (e - wall_le);
              //double i = ki * (e + le);
              double offset = (p + d);
              
              wall_le = e;  // le= last error
    	   
             return offset;    
             
             
                  

}



// function for the gripper
void Gripper(Motor **linear, int dir, int current_pos)
    {
       linear[0]->setPosition(current_pos*0.001+dir*0.001);
       linear[1]->setPosition(current_pos*0.001+dir*0.001);
    }
    
    
 //--------------------------------------------------------


//function for the open and close the hand by given postiion
void SetGripperPosition(Robot* robot, Motor **linear, int target_hand_position, int *current_hand_position)
  {

    while (robot->step(TIME_STEP) != -1)
     {   
            
    //Horizontal Move Control of the hand
      if (*current_hand_position<target_hand_position){
          Gripper(linear,1, *current_hand_position);
          *current_hand_position+=1;
        }
       else if (*current_hand_position>target_hand_position){
           Gripper(linear,-1, *current_hand_position);
           *current_hand_position-=1;    
        } 
       else
        break; 

     } 
  }
    
 //------------------------------------------------------------- 


int main(int argc, char **argv) {
            Robot *robot = new Robot();
            // init Motors
  
	for (int i = 0; i < 2; i++)
	{
		motors[i] = robot->getMotor(motorNames[i]);
		motors[i]->setPosition(INFINITY);
		motors[i]->setVelocity(0.0);
	}
    
    
            // initialize sensors
    
            //object
            DistanceSensor *ir[11];
            
            //name
            char sensorNames[11][8] = {
            "ir0", "ir1", "ir2", "ir3", "ir4",
            "ir5", "ir6", "ir7", "lm", "rm"
            };
            
            //enable the sensors to get measurements
            for (int i = 0; i < 10; i++) {
            ir[i] = robot->getDistanceSensor(sensorNames[i]);
            ir[i]->enable(TIME_STEP);
            }
            
            
           DistanceSensor* us[3];
          char dsNames[3][20] = { "left_ultrasonic", "right_ultrasonic","front_ultrasonic"};
          
          //enable the sensors to get measurements
          for (int i = 0; i < 3; i++) {
          us[i] = robot->getDistanceSensor(dsNames[i]);
          us[i]->enable(TIME_STEP);
          
          }
          
          
          
                 /*
                  
                   //0--50
                // int target_hand_position = 20;
                
                
                  //getting front__sensor
                  Sensor * front_upper = robot-> getSensor("upper_us");
                  front_upper ->enable(TIME_STEP);
                 //-------------------------------------------------------------- 
                  
                  
                //getting postion of gripper  
                   for (int i = 0; i < 2; i++)
                    {
                       linear[i] = robot->getMotor(linearNames[i]);
                       linear[i]->setPosition(0.0);
                    }  
                    
                //----------------------------------------
                  
                      //Hand Motors
                
                  //---- for servo motor ---- // --- to lift the box --//
                  
                  
                  //not used yet
                  
                    servo =  robot->getMotor(servoName);
                    servo->setPosition(0.0);
                    
                    
                    //-------------------------------
                  // get pointers to the two  sensors
                    Sensor *topSensor = robot->getSensor("top_dis");
                    Sensor *bottomSensor = robot->getSensor("bottom_dis");
                    topSensor ->enable(TIME_STEP);
                    bottomSensor ->enable(TIME_STEP);
                    
                    
                    //camera-----------------------
                    Camera* camera = robot -> getCamera("CAM");
                    camera->enable(10); // enable camera with a 10ms refresh rate
                   
                    //-----------------------------------------*/
                          
                          
          
          
	while (robot->step(TIME_STEP) != -1)
	{
		if (break_main_loop)
		{
			motors[0]->setVelocity(0.0);
			motors[1]->setVelocity(0.0);
			break;
		}
	

		switch (state)
		{
              		case 0:     //starting box
              		std::cout<<"success"<<std::endl;
                        		forward();
                        		for (int i = 0; i < 10 ; i++){
                                              sensorValues[i] = ir[i]->getValue();
                                              }
                                            IR_THRESHOLD=500; 
                        		IR_panel=read(IR_THRESHOLD); // call a function to get out put as binary values from the IR array
                        		
                        		if (IR_panel != "00000000")//white
                        		{
                          		     stop();
                          		     IR_THRESHOLD=950;
                          		     
                              		     state=1;
                              		     break;}
                              	break;
                              	
                              	
                        	case 1:     //line following
                                      	std::cout<<"success1"<<std::endl;
                                      	for (int i = 0; i < 10 ; i++){
                                              sensorValues[i] = ir[i]->getValue();
                                              }
                                              
                                             IR_panel=read(IR_THRESHOLD); 
                                              
                        		if (IR_panel == "11111111" && substate==0)//black
                        		{
                          		     stop();
                          		     state=2;
                                  	     break;}
                                  	     
                                  	if (IR_panel == "00000000" && substate==1)//T junction of red & blue
                                  	{  stop();
                                      	
                          		     state=3;
                                  		//---------detecting colour---------//
                                                 leftValue = ir[0]->getValue();
                                                 rightValue = ir[7]->getValue();    
                                  	     break;}
                                  	     
                                  	     
                                              if (IR_panel == "00000000" && substate==3)//chess board
                                  	{  stop();
                                      	
                          		     state=5;
                     
                                  	     break;}
                                  	     
                                  	if (substate==2 && IR_panel == "00001111")
                                  	{
                                    	    IR_panel=read(); 
                                    	    if (IR_panel == "00001111")
                                        	        {rotate="left";
                                                     state =4;
                                                     break;}
                                                else
                                                  IR_panel=read(IR_THRESHOLD);}
                                                  
                                            if (substate==2 && IR_panel == "11110000")
                                  	{
                                    	    IR_panel=read(); 
                                    	    if (IR_panel == "11110000")
                                        	        {rotate="right";
                                                     state =4;
                                                     break;}
                                                else
                                                  IR_panel=read(IR_THRESHOLD);}
                                  	     
                                  	     
                                  	
                                  	     
                                  	else{
                                      	double offset = PID_calc_linefollow(); //get the offset by calling pre defined function
                          
                                                      //---------------------set motor speed values to minimize the error------------------------
                                                      
                                              //std::cout<<"offset"<<offset<<std::endl;
                                              
                                              double left = baseSpeed - offset;
                                              double right = baseSpeed + offset;
                                                      
                                                      //---call a function to map the above speds within its maximum & minimum speed---
                                                      
                                               double leftSpeed = Mdriver(left);
                                               double rightSpeed = Mdriver(right);
                                                      
                                                      
                                               mleft = leftSpeed;
                                               mright = rightSpeed;
                                                      
                                                      //----------------------pass the speeds to the motor for run------------------------------
                                                      
                                               motors[0]->setVelocity(mleft);
                                               motors[1]->setVelocity(mright);
                                                    
                                                    //-------------print the sensor outputs from the IR array & current offset-----------------
                                             /*std::cout<<"ir0 = "<<sensorValues[0]<<"  ";
                                             std::cout<<"ir1 = "<<sensorValues[1]<<"  ";
                                             std::cout<<"ir2 = "<<sensorValues[2]<<"  ";
                                             std::cout<<"ir3 = "<<sensorValues[3]<<"  ";
                                             std::cout<<"ir4 = "<<sensorValues[4]<<"  ";
                                             std::cout<<"ir5 = "<<sensorValues[5]<<"  ";
                                             std::cout<<"ir6 = "<<sensorValues[6]<<"  ";
                                             std::cout<<"ir7 = "<<sensorValues[7]<<std::endl;
                                                        
                                             std::cout<<" offset : "<<offset<<std::endl;*/
                                             
                                             }
                                             
                                             break;
                                           
                               case 2:      //wall following
                                             for (int i = 0; i < 10 ; i++){
                                              sensorValues[i] = ir[i]->getValue();
                                              }
                                              IR_THRESHOLD=500; 
                              		  IR_panel=read(IR_THRESHOLD); // call a function to get out put as binary values from the IR array
                        		
                        		if (IR_panel == "00000000" && substate==0)//white
                        		{
                          		     stop();
                            		     state=0;
                            		     substate=1;
                              		     break;}  
                              		 
                              		 
                              		 else{
                                                  for (int i = 0; i < 3 ; i++){
                                                  UltraSonics[i] = us[i]->getValue();
                                                  }
                                            
                                              
                                                  std::cout<<"left = "<<UltraSonics[0]<<left<<std::endl; 
                                                  std::cout<<"right = "<<UltraSonics[1]<<right<<std::endl; 
                                                        
                                                
                                                 
                                                 for (int i=0;i<2;i++){
                                                  
                                                  if (UltraSonics[i]<1800)
                                                  {
                                                  double UV_read= UltraSonics[i];
                                                  double offset = PID_calc_wallfollowing(UV_read);
                                                  
                                                
                                        	   
                                                  double left = baseSpeed + pow(-1,i)*offset;
                                                  
                                                  double right = baseSpeed - pow(-1,i)*offset;
                                                  double leftSpeed = Mdriver(left);
                                                  double rightSpeed = Mdriver(right);
                                                
                                                 mleft = leftSpeed;
                                                 mright = rightSpeed;
                                               
                                                            //----------------------pass the speeds to the motor for run------------------------------
                                     
                                                 motors[0]->setVelocity(mleft);
                                                 motors[1]->setVelocity(mright);
                                                 //std::cout<<offset<<" left spped="<<mleft<<"right speed ="<<mright<<std::endl;
                                                 }
                                                 else{
                                                 motors[0]->setVelocity(MAX_SPEED);
                                                 motors[1]->setVelocity(MAX_SPEED); 
                                                 }
                                                 }
                                             }break;
                               case 3:
                                                             //choose red/blue line
                                             substate=2;
                                             if (leftValue > 800)
                                                        leftValue = 1; // blue in left
                                             else
                                                        leftValue = 0;
                                                    
                                                    
                                                    //double  rightMostValue = rightMost->getValue();
                                              if (rightValue > 800)
                                                        rightValue = 1;   // blue in right
                                             else
                                                    rightValue = 0;
                                                    
                                                    
                                              std::cout<<"color is "<< color_var <<std::endl;
                                              if (color_var ==0)
                                                        {IR_THRESHOLD=600;
                                                        std::cout<<"color is................. "<<color_var <<std::endl;
                                                       std::cout<<"im following red line.................."<<std::endl;}            
                                               if (color_var ==1)
                                                        {IR_THRESHOLD=950; 
                                                        std::cout<<"color is.................... "<<color_var <<std::endl;
                                                        std::cout<<"im following blue line.................."<<std::endl;}  
                                             
                                                    
                                            
                                                        
                                               //turn left
                                              if ((color_var==0 && leftValue == 0) || (color_var==1 && leftValue == 1))
                                              
                                                {
                                                
                                                double angle = M_PI * 1.7; 
                                                rotateRobot(robot,motors, angle);
                                                
                                                  stop();
                                                  state=1;
                                                  break;}
                                              
                                              //turn right
                                              if ((color_var==0 && rightValue == 0) || (color_var==1 && rightValue == 1))
                                              {
                                                double angle = M_PI * 0.3; 
                                                rotateRobot(robot,motors, angle);

                                                  stop();
                                                  state=1;
                                                  break;}

                                              
                                               break;
                                    
                             case 4:
                                               //turn left/right
                                               
                                               
                                               //turn left
                                              IR_THRESHOLD =800;
                                              if (rotate=="left")
                                              {double angle = M_PI * 1.7; 
                                                rotateRobot(robot,motors, angle);
                                                state=1;
                                                substate=3;
                                                break;} 
                                
                                                  //turn right      
                                                if (rotate=="right")
                                              {double angle = M_PI * 0.3; 
                                                rotateRobot(robot,motors, angle);
                                                state=1;
                                                substate=3;
                                                break;} 
                                             break;
                          
                          case 5:
                          /*
                                              
                                                for (int i = 0; i < 10 ; i++){
                                                      sensorValues[i] = ir[i]->getValue();
                                                      }
                                            
                                            
                                             double  leftMostValue = sensorValues[8];
                                             double  rightMostValue = sensorValues[9]; 
                                            
                                            
                                            
                                            
                                            
                                            //for counting----------------------
                                            if (front_dir==0){
                                              //ir
                                              if(abs((leftMostValue-previous_ir))>100){
                                                previous_ir = leftMostValue;
                                                count+=1;
                                              
                                              }
                                            }
                                            
                                            else{
                                            if(abs((leftMostValue-previous_ir))>100){
                                              previous_ir = leftMostValue;
                                              count+=1;
                                            
                                              count -=1;
                                              }
                                            
                                            }
                                            
                                            
                                            
                                               double _to_other = front_upper->getValue();
                                              
                                              if ( _to_other<3500){
                                              
                                                  stop();
                                                
                                                substage=1;
                                               
                                            
                                            
                                            
                                            }
                                             
                                              
                                            
                                            
                                             
                                             
                                             
                                             
                                             if (substage == 0){ 
                                          
                                          
                                             
                                             if ((current_hand_position== 45) or (king_found)) { 
                                             
                                             
                                             
                                             
                                          //getting values
                                               topsensorvalue = topSensor->getValue();
                                               bottomsensorvalue = bottomSensor -> getValue(); 
                                               std::cout << "top" << topsensorvalue << std::endl; 
                                               std::cout << "bottom" << bottomsensorvalue << std::endl; 
                                                 
                                             //when arm is holding the box
                                               if (topsensorvalue<1000){   //king is there in row or collum
                                                  std::cout << "maby king" << std::endl; 
                                                  
                                                  
                                                  //...............detecting color...........................................
                                                  
                                                  const unsigned char* imageData = camera->getImage(); // get image data
                                                  int width = camera->getWidth(); // get image width
                                                  int height = camera->getHeight(); // get image height
                                                  int blackPixels = 0;
                                                  int whitePixels = 0;
                                              
                                                    // loop through each pixel in the image
                                                    for (int y = 0; y < height; y++) {
                                                      for (int x = 0; x < width; x++) {
                                                        int pixelIndex = (y * width + x) * 4; // each pixel has 4 bytes: RGBA
                                                        int r = imageData[pixelIndex];
                                                        int g = imageData[pixelIndex + 1];
                                                        int b = imageData[pixelIndex + 2];
                                                  
                                                  
                                                      if (r < 70 && g < 70 && b <70 ) {
                                                        blackPixels++;
                                                      } else if (r > 70 && g > 70 && b >70) {
                                                        whitePixels++;
                                                      }
                                                    }
                                                  }
                                              //................................................................................
                                                      if (blackPixels > whitePixels) {
                                                        printf("Black detected!\n");
                                                        
                                                        
                                                      } else if (whitePixels > blackPixels) {
                                                        printf("White detected!\n");
                                                        
                                                        
                                                        if (abs(topsensorvalue-bottomsensorvalue)<20){
                                                 
                                                          std::cout << "stop" << std::endl;
                                                          king_found = true;
                                                     
                                                            stop();
                                                       
                                                       
                                                          SetGripperPosition(robot, linear, 70, &current_hand_position);
                                                          
                                                          
                                          
                                                       
                                                       }
                                                       
                                             
                                                        
                                          
                                                 //king detect
                                                 
                                                  //colour detect
                                                 
                                                   //if color ==white {
                                                   //substage 1} 
                                                 
                                                 
                                                 }
                                                 
                                               }
                                               else{
                                                 forward();
                                               }
                                             
                                             
                                              
                                               }
                                             
                                             //to grab the box....................................................................................
                                               else{
                                                   //for detcting box
                                                  front_ = us[2]->getValue();
                                                
                                                  if (front_> 300){
                                                    leftMotor -> setVelocity(MAX_SPEED);
                                                    rightMotor -> setVelocity(MAX_SPEED);
                                                    SetGripperPosition(robot, linear, 65, &current_hand_position);
                                                    
                                                    
                                                    }
                                                   else{
                                                    leftMotor -> setVelocity(0);
                                                    rightMotor -> setVelocity(0);
                                                    SetGripperPosition(robot, linear, 45, &current_hand_position);
                                                    
                                                    
                                                 // ..................................................................................................
                                               }
                                               }
                                              }
                                               
                                               
                                              
                                              
                                              
                                              
                                               if (substage==2){          // code after check mate
                                               
                                                    forward();
                                                    }
                                               
                                               
                                                  
                                                
                                                if (count==0){
                                                  substage= 3; // stage to finding the door
                                                }*/
                                            
                                                                                         stop();
                                                                                         break;
                            
                                                 
                            }
                            } 
	// cleanup
	delete robot;
	return 0;	
}
                 