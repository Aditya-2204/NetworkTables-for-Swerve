package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Drivetrain extends SubsystemBase {
    private final Pigeon2 pigeonGyro;  // Gyro sensor for angle measurement
    private final SwerveModule[] swerveModules;

    public SwerveModule frontLeftModule;
    public SwerveModule frontRightModule;
    public SwerveModule backLeftModule;
    public SwerveModule backRightModule;

    // NetworkTable to store the robot's data
    private final NetworkTable robotTable;

    public Drivetrain() {
        this.pigeonGyro = new Pigeon2(1);
        SwerveModule frontLeftModule = new SwerveModule(1, 2, 3, 4, pigeonGyro, "FrontLeft");
        SwerveModule frontRightModule = new SwerveModule(5, 6, 7, 8, pigeonGyro, "FrontRight");
        SwerveModule backLeftModule = new SwerveModule(9, 10, 11, 12, pigeonGyro, "BackLeft");
        SwerveModule backRightModule = new SwerveModule(13, 14, 15, 16, pigeonGyro, "BackRight");
        
        this.swerveModules = new SwerveModule[] {
            frontLeftModule,
            frontRightModule,
            backLeftModule,
            backRightModule
        };

        // Initialize the NetworkTable to report robot's data
        robotTable = NetworkTableInstance.getDefault().getTable("RobotData");
    }

    // Update NetworkTables with current velocity and angle
    public void updateDashboard() {
        double robotAngle = pigeonGyro.getYaw().getValueAsDouble();  // Get the robot's current angle from the gyro
        double velocityX = 0.0;
        double velocityY = 0.0;

        // Aggregate the velocities from all swerve modules
        for (SwerveModule module : swerveModules) {
            velocityX += module.getVelocity().getX();  // Summing velocities for simplicity
            velocityY += module.getVelocity().getY();
        }

        // Send angle and velocity to NetworkTables
        robotTable.getEntry("robotAngle").setDouble(robotAngle);
        robotTable.getEntry("velocityX").setDouble(velocityX);
        robotTable.getEntry("velocityY").setDouble(velocityY);
    }

    public void setDrive(double forward, double strafe, double rotation) {
        // This method will control the swerve modules and update their velocities
        for (SwerveModule module : swerveModules) {
            module.setDesiredState(forward, strafe, rotation);
        }
    }

    @Override
    public void periodic() {
        // This method will be called periodically (i.e., every 20ms)
        System.out.println("FL Angle: " + frontLeftModule.getAngle() + " | FL Velocity: " + frontLeftModule.getVelocity());
        System.out.println("FR Angle: " + frontRightModule.getAngle() + " | FR Velocity: " + frontRightModule.getVelocity());
        System.out.println("BL Angle: " + backLeftModule.getAngle() + " | BL Velocity: " + backLeftModule.getVelocity());
        System.out.println("BR Angle: " + backRightModule.getAngle() + " | BR Velocity: " + backRightModule.getVelocity());
    }
}
