// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;



public class LiftSubsystem extends SubsystemBase {
  private SparkMax liftR,liftL;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  
  /** Creates a new AlgaeSubsystem. */
  public LiftSubsystem() {
    liftR = new SparkMax(LiftConstants.LiftMotorIDR, MotorType.kBrushless);
    liftL = new SparkMax(LiftConstants.LiftMotorIDL, MotorType.kBrushless);

    closedLoopController = liftL.getClosedLoopController();
    encoder = liftL.getEncoder();

    

  

     SparkMaxConfig LiftRConfig = new SparkMaxConfig();

     SparkMaxConfig LiftLConfig = new SparkMaxConfig();


        // Motor Inverts and Neutral Mode 
        
        
        LiftLConfig.inverted(true)
                        .idleMode(IdleMode.kBrake);

                        LiftLConfig.closedLoop
                        .p(LiftConstants.LiftP)
                        .i(LiftConstants.LiftI)
                        .d(LiftConstants.LiftD)
                        
                        .outputRange(.10, .15);
                      



                        
                        

                      
                        
                        
                        
                        
        

        LiftRConfig
                    
                      .idleMode(IdleMode.kBrake)
                       
                       .follow(liftL, true);
                       
                      
                      
                      
                      
                      


                    liftL.configure(LiftLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                    liftR.configure(LiftRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        

        //liftR.isFollower(true);
                      
  }


  public void setLiftSpeed(double LiftSpeed){
    if (getPositionInches() <= 40 && LiftSpeed > 0){
      liftL.set(LiftSpeed);
      liftR.set(LiftSpeed);
    }

    
    else if (getPositionInches() >= 0 && LiftSpeed < 0){
        liftL.set(LiftSpeed);
        liftR.set(LiftSpeed);
    }

    else{
      liftL.set(0);
      liftR.set(0);

    }
    
  }

  public double getPositionInches() {
    double test = encoder.getPosition() * LiftConstants.gearRatio * (2.074 * Math.PI) * 2;
    //((Math.PI * LiftConstants.sprocketDiameter) * LiftConstants.gearRatio));
    
  
    System.out.println(test);
    return test;
   
  
}

// public void setPos(double posin){
//   closedLoopController.setReference((posin * 48 * 22)/(14 * 2.074 * Math.PI), ControlType.kPosition);

// }

  public void setPos(double pos){
      while(getPositionInches() <= pos){
        liftL.set(.2);
    }
    
  //   if(getPositionInches() != pos){ 
  //     liftL.set(.2);
  //  }

  //  else{
  //   liftL.set(0);
  //  }
   }


    // public void reachGoalFeet(double goalFeet) {
    //     // Convert  feet to rotations
    //     double targetRotations = (goalFeet*12) / ((Math.PI * LiftConstants.sprocketDiameter) * LiftConstants.gearRatio);

    //     // Set the target position using the closed loop controller.
    //     closedLoopController.setReference(targetRotations, SparkMax.ControlType.kPosition);
    // }

    // public Command setGoalFeet(double goalFeet) {
    //     return run(() -> reachGoalFeet(goalFeet));
    // }

    // public Command setElevatorHeightFeet(double heightFeet) {
    //     return setGoalFeet(heightFeet).until(() -> aroundHeightFeet(heightFeet));
    // }

    // public boolean aroundHeightFeet(double heightFeet) {
    //     return aroundHeightFeet(heightFeet, LiftConstants.kElevatorDefaultTolerance);
    // }

    // public boolean aroundHeightFeet(double heightFeet, double toleranceFeet) {
    //     return MathUtil.isNear(heightFeet, getPositionInches(), toleranceFeet);
    // }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator height (in)", getPositionInches());

    
    if(getPositionInches() <= -1.5 || getPositionInches() >= 41.5 ){
      
      liftL.set(0);
    }

  }

  
}
