
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//OKYERE UPDATE 1
// AUTO MOVE 1 BLOCK: COMPLETE. PRESS BUTTON 9 TO GO FOWARD AT DICKO MODE SPEED FOR APPROX 2 SECONDS

// CONTROL ROBOT WITH ONE CONTROLLER:
// COMPLETE BUT REQUIRES REVIEW. MOST LIKELY CORRECT
// NOW USES MAGNITUDE OF JOYSTICK TO DETERMINE SPEED IN SPECIFIC RANGES. SIMILAR TO A STICKSHIFT
// OPERATES BASED OFF OF ANGLE AND NOT COORDINATES
// SLOW/NORMAL/DICKO MODE STILL EXIST

// ARM MOVEMENT TO SPECIFIC POINTS: CURRENTLY UNSURE ABOUT
// - WHICH ANGLES AND HOW MANY ANGLES TO HAVE ARM AT
// - WHAT SPEED THE ARM IS CAPABLE OF/NEEDS TO OPERATE AT
// WILL WING IT, BUT PROBABLY WONT WORK.
// SHIFT FN F5 TO DEPLOY CODE
package frc.robot.commands;
import java.util.Date;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import java.lang.*;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class DriveManuallyCommand extends Command {
  public double attacklefty;
  public double attackrighty;
  



  public DriveManuallyCommand() {
    // Use requires() here to declare subsystem dependencies
     requires(Robot.driveSubsystem);
    //constructor function
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Robot.driveSubsystem.setSetPoint();
    //rest gyoro or stuff put here
  }

 
  /*This code allows you to switch driving mode using button 7. 
  each press cycles through mode 1 and 2. it automatically starts with in TankMode*/
  //there are currently two control
  int modes=3;            //methods:Tank and Car Controls.
  int DrivingMode;        //if a new mode for driving is created,
  int CyclePresses=0;     //add 1 to modes and change the switch statement
  int SwitchPressTime=0;
  double left=0.0;
  double right=0.0;
  double speedSetting=0.7;

  int partThreeTask=1;
  int assignedSpins;
  Color assignedColor;

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
    boolean single = true;
    if (single){   

////////////////////////////////////////////////                                                                                                                                      //////////////////////////////////////////////
      /*THIS IS CONTROL SETUP #1: TANK CONTROLS
      THIS IS UNDER THE ASSUMPTION THAT THE JOYSTICK ANGLE
      IS DETERMINED AS SO:
               0

         270        90
                  
              180
      IF NOT, SIMPLY CHANGE THE VALUES TO RESEMBLE:
        JOYSTICK FOWARD:        BOTH WHEELS MOVE FOWARD
        JOYSTICK BACKWARD:      BOTH WHEELS MOVE BACK
        JOYSTICK LEFT:          LEFT WHEELS MOVE BACK.        RIGHT WHEELS MOVE FOWARD
        JOYSTICK RIGHT:         RIGHT WHEELS MOVE BACK.       LEFT WHEELS MOVE FOWARD
        JOYSTICK FOWARD-RIGHT:  LEFT WHEELS MOVE FOWARD.      RIGHT WHEELS DONT MOVE
        JOYSTICK FOWARD-LEFT:   RIGHT WHEELS MOVE FOWARD.     LEFT WHEELS DONT MOVE
        JOYSTICK BACK-RIGHT:    LEFT WHEELS MOVE BACKWARD.    RIGHT WHEELS DONT MOVE
        JOYSTICK BACK-LEFT:     RIGHT WHEELS MOVE BACKWARD.   LEFT WHEELS DONT MOVE

      */
      double TankLeft;
      double TankRight;
      double direction = Robot.oi.stick.getDirectionDegrees();
      if(direction>337.5||direction<=22.5){ //range of 337.5 to 22.5 degrees. middle is 0 
        TankLeft=1;
        TankRight=1;
      }
      else
        if(direction<=67.5){ //range of 22.5 to 67.5.5 degrees. middle is 45
          TankLeft=1;
          TankRight=0;
        }
      else
        if(direction<=112.5){ //range of 67.5 to 112.5 degrees. middle is 90
          TankLeft=1;
          TankRight=-1;
        }
      else
        if(direction<=157.5){ //range of 112.5 to 157.5 degrees. middle is 135
          TankLeft=0;
          TankRight=-1;
        }
      else
        if(direction<=202.5){ //range of 157.5 to 202.5 degrees. middle is 180
          TankLeft=-1;
          TankRight=-1;
        }
      else
        if(direction<=247.5){ //range of 202.5 to 247.5 degrees. middle is 225
          TankLeft=0;
          TankRight=-1;
        }
      else
        if(direction<=292.5){ //range of 247.5 to 292.5 degrees. middle is 270
          TankLeft=-1;
          TankRight=1;
        }
      else{ //range of 292.5 to 337.5 degrees. middle is 315
          TankLeft=0;
          TankRight=1;
        }
      TankLeft*=Robot.oi.stick.getMagnitude();
      TankRight*=Robot.oi.stick.getMagnitude();
    //////////////////////////////////////////////////////////////////////////////////////////////////
        /*SETUP #2. THIS REQUIRES 2 CONTROLLERS. THE FIRST CONTROLLER USES THE X-AXIS TO DETERMINE
        THW ANGLE. THE SPEED IS DETERMINED USING THE Y AXIS OF THE SECOND CONTROLLER. REQUIREMENTS TO
        UNDERSTAND AND CHANGE THIS:
        -TRIG FUNCTIONS
        -THE ANGULAR SETUP OF THE JOYSTICKS
              THIS IS UNDER THE ASSUMPTION THAT THE JOYSTICK ANGLE
              IS DETERMINED AS SO:
               90

         0          180
                  
              270
        */
      double Xcoordinate = Robot.oi.stick.getRawAxis(0);
      double CarAngle    = Math.cos(Math.PI*Xcoordinate);
      double CarLeft;
      double CarRight;
      if(Xcoordinate<0){
        CarLeft = CarAngle;
        CarRight= 1;
      }
      else{
        CarLeft = 1;
        CarRight= CarAngle;
      }
      CarLeft*=Robot.oi.rightStick.getRawAxis(1);
      CarRight*=Robot.oi.rightStick.getRawAxis(1); 
  //////////////////////////////////////////////////////////////////////////////////////////////
      /*SETUP #3. THIS REQUIRES 2 CONTROLLERS. THE FIRST CONTROLLER USES THE Y-AXIS TO DETERMINE
      THE SPEED OF THE LEFT SET OF WHEELS WHILE THE SECOND CONTROLLER'S Y-AXIS DETERMINES THE 
      SPEED OF THE RIGHT SET OF WHEELS. REQUIREMENTS TO
      UNDERSTAND AND CHANGE THIS:
      -THE AXIS RANGE OF THE CONTROLLERS
            THIS IS UNDER THE ASSUMPTION THAT THE JOYSTICK  Y-AXIS
            IS DETERMINED AS SO:
        Y=  -1          1
            
             0          0
      
             1          1
      THE NAME WAS CHOSEN DUE TO ITS RESEMBLANCE TO HOW KIDS IN A TRENCHCOAT WOULD MOVE*/
      double TrenchcoatLeft=-Robot.oi.rightStick.getRawAxis(1);
      double TrenchcoatRight=-Robot.oi.stick.getRawAxis(1);
  //////////////////////////////////////////////////////////////////////////////////////////////
  boolean buttonStart= Robot.oi.stick.getRawButton(10);
  boolean buttonBack= Robot.oi.stick.getRawButton(9);
  int pov = Robot.oi.stick.getPOV();        
    



  boolean SwitchDriveMode=Robot.oi.stick.getRawButton(7);
  if(SwitchDriveMode){
    SwitchPressTime++;
  }
  if(SwitchPressTime>=10){
    CyclePresses++;
  }
  SwitchDriveMode=false;
    DrivingMode=CyclePresses%modes;

//Change this code if there is a new mode created
// case 3:
// left = DifferentLeft
// right= DifferentRight
// case 4:
// and so on.
  switch(DrivingMode){
    case 0:
      left=CarLeft;
      right=CarRight;
      break;
    case 1:
      left=TankLeft;
      right=TankRight;
      break;
    case 2:
      left=TrenchcoatLeft;
      right=TrenchcoatRight;
      break;
  }           
  /////////////////////////////////////////////////////////////////////////////////////////////
          
        /*THIS IS THE CODE FOR THE AUTO FUNCTIONS. SO FAR THE THREE AUTO FUNCTIONS ARE:
        ARM MOVE UP FOR 2 SECONDS
        ARM MOVE DOWN FOR TWO SECONDS
        ROBOT GO FOWARD FOR 5 SECONDS */
        
      boolean AutoMoveButton = Robot.oi.stick.getRawButton(9);
      boolean button10 = Robot.oi.rightStick.getRawButton(10);
      boolean button11 = Robot.oi.rightStick.getRawButton(11);
      Robot.driveSubsystem.AutoFunctions(AutoMoveButton, button10, button11);
 
  /////////////////////////////////////////////////////////////////////////////////////////////
      //Buttons for Game Controller

      boolean buttonXFast = Robot.oi.stick.getRawButton(1); //supa fast
      boolean buttonYSlow = Robot.oi.stick.getRawButton(4); //slow down
      boolean buttonA = Robot.oi.stick.getRawButton(2); //deals with arm speed
      boolean buttonBRegular = Robot.oi.stick.getRawButton(3); //regular speed

/////////////////////////////////////////////////////////////////////////////
      if(Robot.oi.rightStick.getRawButton(6))
        Robot.driveSubsystem.manuallySpinWheel();
      
      if(Robot.oi.rightStick.getRawButton(1)){
        partThreeTask++;
        partThreeTask%=2;
      }
      if(partThreeTask==1){
        if(Robot.oi.rightStick.getRawButton(2))
          assignedColor = ColorMatch.makeColor(0.143, 0.427, 0.429); //blue
        
        else if(Robot.oi.rightStick.getRawButton(3))
          assignedColor = ColorMatch.makeColor(0.197, 0.561, 0.240); //green
        
        else if(Robot.oi.rightStick.getRawButton(4))
          assignedColor = ColorMatch.makeColor(0.561, 0.232, 0.114); //red
        
        else if(Robot.oi.rightStick.getRawButton(5))
          assignedColor = ColorMatch.makeColor(0.361, 0.524, 0.113); //yellow
        
        Robot.driveSubsystem.spinWheel(assignedColor);
      }else{
        if(Robot.oi.rightStick.getRawButton(2))
          assignedSpins=3;

        else if(Robot.oi.rightStick.getRawButton(3))
          assignedSpins=4;

        else if(Robot.oi.rightStick.getRawButton(4))
          assignedSpins=5;
        
        Robot.driveSubsystem.spinWheel(assignedSpins);
      }
/////////////////////////////////////////////////////////////////////////////
      if(buttonXFast)
        speedSetting=0.9;
      else if(buttonYSlow)
        speedSetting=0.5;
      else if(buttonBRegular)
        speedSetting=0.7;

        

        Robot.driveSubsystem.manualDrive(left, right,
         pov, buttonA, speedSetting, buttonBack);
      
      }
      
    }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
    //automanous command
  }

  // Called once after is Finished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
