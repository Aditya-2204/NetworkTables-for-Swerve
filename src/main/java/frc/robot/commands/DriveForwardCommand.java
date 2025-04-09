package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardCommand extends Command {
    private final Drivetrain Drivetrain;
    private double timeElapsed;
    private final double driveDuration = 2.0; // Move forward for 2 seconds

    public DriveForwardCommand(Drivetrain Drivetrain) {
        this.Drivetrain = Drivetrain;
        addRequirements(Drivetrain);  // Declare the subsystem required for this command
    }

    @Override
    public void initialize() {
        timeElapsed = 0.0;
    }

    @Override
    public void execute() {
        Drivetrain.setDrive(0.5, 0, 0);  // Move forward at half speed
        timeElapsed += Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return timeElapsed >= driveDuration;  // End command after 2 seconds
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.setDrive(0, 0, 0);  // Stop the robot when done
    }
}
