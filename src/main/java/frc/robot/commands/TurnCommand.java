package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;


public class TurnCommand extends Command {
    private final Drivetrain Drivetrain;
    private double timeElapsed;
    private final double turnDuration = 1.0; // Rotate for 1 second

    public TurnCommand(Drivetrain Drivetrain) {
        this.Drivetrain = Drivetrain;
        addRequirements(Drivetrain);  // Declare the subsystem required for this command
    }

    @Override
    public void initialize() {
        timeElapsed = 0.0;
    }

    @Override
    public void execute() {
        Drivetrain.setDrive(0, 0, 0.5);  // Rotate at half speed
        timeElapsed += Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return timeElapsed >= turnDuration;  // End command after 1 second
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.setDrive(0, 0, 0);  // Stop the robot when done
    }
}
