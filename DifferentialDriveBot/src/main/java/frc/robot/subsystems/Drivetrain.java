// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//Drivetrain imports
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
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
  private final CANSparkMax leftFollower;

  private final CANSparkMax rightLeadMotor;
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

    //Initialize Motors
    leftLeadMotor = new CANSparkMax(DrivetrainConstants.LEFT_LEAD_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushlesss);
    leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER_MOTOR_CAN_ID,CANSparkMaxLowLevel.MotorType.kBrushless);

    rightLeadMotor = new CANSparkMax(DrivetrainConstants.RIGHT_LEAD_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftLeadMotor.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    //Setting Up Left And Right Encoders
    leftEncoder = leftLeadMotor.getEncoder();
    leftEncoder.setPositionConversionFactor();
    leftEncoder.setVelocityConversionFactor();

    rightEncoder = rightLeadMotor.getEncoder();
    rightEndocer.setPositionConversionFactor();
    rightEncoder.setVelocityConversionFactor();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftLeadMotor.setInverted(true);
    rightLeadMotor.setInverted(false);

    //Sets All Motors To Break Mode
    leftLeadMotor.setIdleModer(CANSparkMax.IdleMode.kBrake);
    leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLeadMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //Follow Lead Motors
    leftFollower.follow(leftLeadMotor, DrivetrainConstants.LEFT_MOTOR_INVERT);
    rightFollower.follow(rightLeadMotor, DrivetrainConstants.RIGHT_MOTORS_INVERT);


    Constants.SPARK_LIST.add(leftLeadMotor);
    Constants.SPARK_LIST.add(leftFollower);
    Constants.SPARK_LIST.add(rightLeadMotor);
    Constants.SPARK_LIST.add(rightFollower);

    //Initialize Motor Groups
    leftMotors = new MotorControllerGroup(leftFollower, leftLeadMotor);
    rightMotors = new MotorControllerGroup(rightFollower, rightLeadMotor);

    //Sets Drive Subsystem
    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }
}
