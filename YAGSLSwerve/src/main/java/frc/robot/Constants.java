// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double deadzone = 0;

    
  }

  public final class SwerveConstants{
    public static double MAX_SPEED = Units.feetToMeters(4.5);

  }

    public static final class LiftConstants{
        
        public static int LiftMotorIDR = 13; 
        public static int LiftMotorIDL = 14; 

         //14 T 1.123
         //22 T 1.757

        //14-22 
        //14 on motor 22 on the shaft to drive 
    

        public static double sprocketDiameter = 2.074;
        public static double sprocketGearRatio = 16/22;
        public static double elevatorGearBoxRatio = 1/48;

        public static double gearRatio = sprocketGearRatio * elevatorGearBoxRatio;

        



        



        public static double LiftP = 1.00;
        public static double LiftI = 1.00;
        public static double LiftD = 1.00;

        public static final double kElevatorDefaultTolerance = Inches.of(1).in(Feet);
        
        
        SparkMaxConfig LiftConfig = new SparkMaxConfig();

        
        
    }

    public static final class CoralConstants{
        
        public static int ElbowID = 15; 
        public static int WristID = 16; 
        public static int CRollerID = 17; 

        public static double elbowP = 0.00;
        public static double elbowI = 0.00;
        public static double elbowD = 0.00;
       
        public static double wristP = 0.00;
        public static double wristI = 0.00;
        public static double wristD = 0.00;
        
        SparkMaxConfig CoralConfig = new SparkMaxConfig();


    }

    public static final class AlgaeConstants{
        
        public static int PivotID = 18; 
        public static int ARollerID = 19; 

        SparkMaxConfig AlgaeConfig = new SparkMaxConfig();

    }

    public final class controllerConstants{

      /*TODO add button bindings */


      /*Driver Constants */

      /*Algae */

      public static int pickUp = 0;
      
      
      /*Operator Constants */   
      public static final double stickDeadband = 0.1;
     
     
     /*Elevator & Coral */
      public static final int elevatorUp = 5;
      public static final int elevatorDown = 6;
      public static final int speedNerf = 3;
      
      public static final int L1 = 2;
      public static final int L2 = 0;
      public static final int L3 = 0;
      public static final int L4 = 0;







  }


  
}
