// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

import static edu.wpi.first.units.Units.Meter;
import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveSubsytem extends SubsystemBase {
  /** Creates a new SwerveSubsytem. */
  
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive  swerveDrive; 
  
  public SwerveSubsytem() {
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED,

        new Pose2d(new Translation2d(
          Meter.of(1),
          Meter.of(4)),
          Rotation2d.fromDegrees(0)
          )
        );
      

      
                                                                      
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public SwerveDrive getSwereDrive() {
    return swerveDrive;
  }

  public void fieldCentric(ChassisSpeeds vel){
    swerveDrive.driveFieldOriented(vel);

  }

  public Command driveFieldCentric(Supplier<ChassisSpeeds> vel){
    return run(() -> {
      swerveDrive.driveFieldOriented(vel.get());
    }
    
  );    

  }
}
