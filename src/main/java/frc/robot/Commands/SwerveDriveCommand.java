package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveCommand extends Command {
    private final SwerveDrive swerveDrive;
    private final double front;
    private final double side;
    private final double rot;

    public SwerveDriveCommand(SwerveDrive swerveDrive, double front, double side, double rot) {
        this.swerveDrive = swerveDrive;
        this.front = front;
        this.side = side;
        this.rot = rot;
        addRequirements(swerveDrive); // Important for scheduler to handle subsystem properly
    }

    @Override
    public void execute() {
        swerveDrive.drive(front, side, rot);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(0, 0, 0); // stop motors
    }
}
