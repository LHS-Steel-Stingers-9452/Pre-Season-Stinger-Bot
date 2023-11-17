// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//Drivetrain imports
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.filter.SlewRateLimiter; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  //Creates Differential Drive
  private final DifferentialDrive drive;

  //Creates Motor variables
  private final CANSparkMax leftLeadMotor;
  private final CANSparkMax rightLeadMotor;

  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;

  //Creates Motor group variables
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  //Creates Motor Encoders
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEndocer;

  // Create Filters For Slew Rate Limiting
  SlewRateLimiter turnFilter;
  SlewRateLimiter driveFilter;


  

  public Drivetrain() {

    driveFilter = new SlewRateLimiter();
    turnFilter = new SlewRateLimiter();



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
