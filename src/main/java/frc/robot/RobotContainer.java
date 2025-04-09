package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.DriveForward;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    private final SwerveDrive swerveDrive = new SwerveDrive();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // joystick bindings if needed
    }

    public Command getAutonomousCommand() {
        return new DriveForward(swerveDrive);
    }
}
