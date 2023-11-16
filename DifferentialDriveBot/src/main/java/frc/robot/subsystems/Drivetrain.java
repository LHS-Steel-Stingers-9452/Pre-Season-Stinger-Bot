// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//Drivetrain imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.filter.SlewRateLimiter; 



public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  //Motor variables
  private final CANSparkMax leftLeadMotor;
  private final CANSparkMax rightLeadMotor;

  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;

  //Motor group variables
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  
  // Create filters for slew rate limiting
  SlewRateLimiter turnFilter;
  SlewRateLimiter driveFilter;
  

  public Drivetrain() {


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
