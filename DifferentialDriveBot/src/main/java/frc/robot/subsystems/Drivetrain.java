// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//Drivetrain imports
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.filter.SlewRateLimiter; 
//math loolololol
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  //Creates Differential Drive
  private final DifferentialDrive drive;

  //private Field2d field;
  private static Drivetrain instance;

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
  private RelativeEncoder rightEncoder;

  //creates Pigeon gyro
  private final Pigeon2 gyro;

  //Creates Forward and Turn Vars For Speed Input
  private double forward;
  private double turn; 

  // Create Filters For Slew Rate Limiting
  SlewRateLimiter turnFilter;
  SlewRateLimiter driveFilter;

  //private final DifferentialDriveKinematics kinematics;
  //private final DifferentialDriveOdometry odometry;


  public Drivetrain() {

    gyro = new Pigeon2(DrivetrainConstants.PIGEON_GYRO_CAN_ID);
    //field = new Field2d();

    //Slew Rate Limiters
    driveFilter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_DRIVE_POSITIVE,
      DrivetrainConstants.SLEW_RATE_DRIVE_NEGATIVE, 0);
    turnFilter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_TURN_POSITIVE,
      DrivetrainConstants.SLEW_RATE_TURN_NEGATIVE, 0);

    //Initialize Motors
    leftLeadMotor = new CANSparkMax(DrivetrainConstants.LEFT_LEAD_MOTOR_CAN_ID,
          CANSparkMaxLowLevel.MotorType.kBrushless);
    leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER_MOTOR_CAN_ID,
          CANSparkMaxLowLevel.MotorType.kBrushless);

    rightLeadMotor = new CANSparkMax(DrivetrainConstants.RIGHT_LEAD_MOTOR_CAN_ID, 
          CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER_MOTOR_CAN_ID, 
          CANSparkMaxLowLevel.MotorType.kBrushless);

    //Restore As A Saftey Measure
    leftLeadMotor.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    //Setting Up Left And Right Encoders
    leftEncoder = leftLeadMotor.getEncoder();
    leftEncoder.setPositionConversionFactor(DrivetrainConstants.DRIVING_ENCODER_POS_FACTOR);
    leftEncoder.setVelocityConversionFactor(DrivetrainConstants.DRIVING_ENCODER_VEL_FACTOR);

    rightEncoder = rightLeadMotor.getEncoder();
    rightEncoder.setPositionConversionFactor(DrivetrainConstants.DRIVING_ENCODER_POS_FACTOR);
    rightEncoder.setVelocityConversionFactor(DrivetrainConstants.DRIVING_ENCODER_VEL_FACTOR);

    //Reset As A Saftery Measure
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftLeadMotor.setInverted(true);
    rightLeadMotor.setInverted(false);

    //Set All Motors To Break Mode
    leftLeadMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLeadMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);

     //Follow Lead Motors
     leftFollower.follow(leftLeadMotor, DrivetrainConstants.LEFT_MOTORS_INVERT);
     rightFollower.follow(rightLeadMotor, DrivetrainConstants.RIGHT_MOTORS_INVERT);

    //Add motors after initilization to SPARK_LIST *Saves Parameters
    Constants.SPARK_LIST.add(leftLeadMotor);
    Constants.SPARK_LIST.add(leftFollower);
    Constants.SPARK_LIST.add(rightLeadMotor);
    Constants.SPARK_LIST.add(rightFollower);

    //Initialize Motor Groups
    leftMotors = new MotorControllerGroup(leftFollower, leftLeadMotor);
    rightMotors = new MotorControllerGroup(rightFollower, rightLeadMotor);

    //Sets Drive Subsystem
    drive = new DifferentialDrive(leftMotors, rightMotors);

    /*
    Math stuff for Auton 
    //Create A Field For Displaying Robot Position Nn The Dashboard
    SmartDashboard.putData("Field", this.field);

    //Math Related Code Goes Here
    //kinumatics
    kinematics = new DifferentialDriveKinematics(DrivetrainConstants.TRACKWIDTH);
    //odometry
    //odometry = new DifferentialDriveOdometry()
    //feedforward
    //PID Controllers
    //Slew Rate Limiters
    driveFilter = new SlewRateLimiter();
    turnFilter = new SlewRateLimiter();
    //Ramsete Controllers
    */


  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double slewForward = driveFilter.calculate(forward);

    drive.arcadeDrive(
      DriverStation.isAutonomous() ? forward: slewForward, 
      turn);
  }

  public void drive(double forward, double turn) {
    this.forward = -forward;
    this.turn = turn;
    drive.feed();
  }

  public static synchronized Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }
  
}