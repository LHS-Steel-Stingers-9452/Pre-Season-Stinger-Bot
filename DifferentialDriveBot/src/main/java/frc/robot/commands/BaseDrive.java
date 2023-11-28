package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DrivetrainConstants;

public class BaseDrive extends CommandBase{
    private final Drivetrain drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier turn;

    public BaseDrive(Drivetrain drive, DoubleSupplier forward, DoubleSupplier turn){
        this.drive = drive;
        this.forward = forward;
        this.turn = turn;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.drive(forward.getAsDouble()*DrivetrainConstants.DRIVE_SPEED_MULT, 
            turn.getAsDouble()*DrivetrainConstants.TURN_SPEED_MULT);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Commands.runOnce(() -> Drivetrain.getInstance().drive(0, 0));
    }

    // Returns true when the command should end.
    // We never want this command to end, so it always returns false
    @Override
    public boolean isFinished() {
        return false;
    }
}
