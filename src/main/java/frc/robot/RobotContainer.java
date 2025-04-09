package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.DriveForwardCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
    // Xbox Controller for controlling the robot
    private final XboxController driverController = new XboxController(0);

    // Pigeon2 Gyro for overall robot heading // Assuming CAN ID 1 for Pigeon2

    // Initialize swerve modules with their motor ports, encoders, and the Pigeon2

    // Create the Drivetrain, passing in all the swerve modules
    private final Drivetrain Drivetrain = new Drivetrain();

    // Constructor to initialize the robot container
    public RobotContainer() {
        // Set up the default command for the Drivetrain
        Drivetrain.setDefaultCommand(new DriveForwardCommand(Drivetrain));

        // Configure button bindings
        configureButtonBindings();
    }

    // Method to configure the button bindings
    private void configureButtonBindings() {
        // Example: bind a button to reset the gyro (Pigeon2)
        new JoystickButton(driverController, XboxController.Button.kA.value);
    }

    // Return the autonomous command
    public Command getAutonomousCommand() {
        // Create a simple autonomous sequence, for example, driving forward for 2 seconds
        return new SequentialCommandGroup(
            new DriveForwardCommand(Drivetrain)
        );
    }
}
