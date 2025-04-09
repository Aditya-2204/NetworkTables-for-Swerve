package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
    private final Drivetrain driveSubsystem;
    private final Joystick joystick;

    public Robot() {
    // Initialize swerve modules with their motor ports, encoders, and the Pigeon2
        driveSubsystem = new Drivetrain();
        joystick = new Joystick(0);  // Using joystick on port 0
    }

    @Override
    public void robotInit() {
        // Initialization code if needed
    }

    @Override
    public void autonomousInit() {
        // Initialize autonomous mode
    }

    @Override
    public void autonomousPeriodic() {
        // Autonomous code for driving forward and turning, for example:
        driveSubsystem.setDrive(0.5, 0, 0);  // Move forward at half speed
        Timer.delay(2);  // Move for 2 seconds
        
        // Rotate 90 degrees in place
        driveSubsystem.setDrive(0, 0, 0.5);
        Timer.delay(1);  // Rotate for 1 second
    }

    @Override
    public void teleopPeriodic() {
        // Read joystick inputs to control robot movement
        double forward = joystick.getY();
        double strafe = joystick.getX();
        double rotation = joystick.getZ();
        
        driveSubsystem.setDrive(forward, strafe, rotation);
    }

    @Override
    public void disabledPeriodic() {
        // Code to run when robot is disabled
    }

    @Override
    public void robotPeriodic() {
        driveSubsystem.updateDashboard();  // Update NetworkTables and SmartDashboard
    }
}
