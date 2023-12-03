// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.ControllerConstants;
//import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.BaseDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  //Subsystems
  private final Drivetrain drivetrain;
  private final XboxController driverController;
  

  //Commands
  private final BaseDrive baseDrive;

  public RobotContainer() {
    configureBindings();

    drivetrain =  Drivetrain.getInstance();
    driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

    baseDrive = new BaseDrive(
      drivetrain,
      () -> MathUtil.applyDeadband(-driverController.getLeftY(), ControllerConstants.DRIVER_FORWARD_DEADBAND),//?moving back, make negative
      () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.DRIVER_TURN_DEADBAND));
    
    drivetrain.setDefaultCommand(baseDrive);

    RobotContainer.incinerateMotors();

  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
  public static void incinerateMotors(){
    Timer.delay(0.25);
    for(CANSparkMax spark : Constants.SPARK_LIST){
      spark.burnFlash();
      Timer.delay(0.005);
    }
    Timer.delay(0.25);
  }
}
