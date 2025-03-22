// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.SwerveSubsytem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsytem drive = new SwerveSubsytem();
 

  private final XboxController driver = new XboxController(0);
      private final Joystick operator = new Joystick(1);

   /* Operator Buttons */
    private final JoystickButton elevatorUp = new JoystickButton(operator, controllerConstants.elevatorUp);
    private final JoystickButton elevatorDown = new JoystickButton(operator, controllerConstants.elevatorDown);
    
    private final JoystickButton L1 = new JoystickButton(operator, controllerConstants.L1);
    private final JoystickButton L2 = new JoystickButton(operator, controllerConstants.L2);
    private final JoystickButton L3 = new JoystickButton(operator, controllerConstants.L3);
    private final JoystickButton L4 = new JoystickButton(operator, controllerConstants.L4);


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



    /* Subsystems */
  
    private final LiftSubsystem l_lift = new LiftSubsystem();
    private final CoralSubsystem c_coral = new CoralSubsystem();
 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drive.setDefaultCommand(fieldCenctricAngleVel);
  }

  SwerveInputStream driveAngleVel = SwerveInputStream.of(drive.getSwereDrive(),
    ()-> driver.getLeftY() * -1, 
    () -> driver.getLeftX() * -1
    )
    
    .withControllerRotationAxis
    (driver::getRightX)
    .deadband(OperatorConstants.deadzone)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

   
   
    SwerveInputStream driveAngle = driveAngleVel.copy().withControllerHeadingAxis(
      driver::getRightX,
      driver::getRightY)
      .headingWhile(true);


      Command fieldCentricAngle =  
      
      drive.driveFieldCentric(driveAngle);

      Command fieldCenctricAngleVel = drive.driveFieldCentric(driveAngleVel);


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
      /* Driver Buttons */

      /*Driver buttons */

      /*Chassis */
     

      
      
       /* Operator Buttons */
     
      /*Elevator&Coral buttons */
      elevatorUp.whileTrue(new InstantCommand(() -> l_lift.setLiftSpeed(-.25)));
      elevatorUp.whileFalse(new InstantCommand(() -> l_lift.setLiftSpeed(0)));

      elevatorDown.whileTrue(new InstantCommand(() -> l_lift.setLiftSpeed(.25)));
      elevatorDown.whileFalse(new InstantCommand(() -> l_lift.setLiftSpeed(0))); 
      
      L1.onTrue(new InstantCommand(() ->l_lift.setPos(6.3))); 
      L1.onFalse(new InstantCommand(() -> l_lift.setLiftSpeed(0)));


      L2.onTrue(new InstantCommand(() ->l_lift.setPos(6.3)));
      L2.onFalse(new InstantCommand(() -> l_lift.setLiftSpeed(0)));


      L3.onTrue(new InstantCommand(() ->l_lift.setPos(6.3)));
      L3.onFalse(new InstantCommand(() -> l_lift.setLiftSpeed(0)));


      L4.onTrue(new InstantCommand(() ->l_lift.setPos(6.3)));
      L4.onFalse(new InstantCommand(() -> l_lift.setLiftSpeed(0)));

      
      
      //-4
  }
    
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
