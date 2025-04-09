package frc.robot.Auton;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrive;

public class DriveForward extends SequentialCommandGroup {
    public DriveForward(SwerveDrive swerveDrive) {
        addCommands(
            new PathPlannerAuto("Goofy")
        );
    }
}
