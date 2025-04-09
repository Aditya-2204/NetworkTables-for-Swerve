package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModule extends SubsystemBase{
    private final SparkMax driveMotor;  // Motor to control the speed of the wheel
    private final SparkMax steeringMotor;  // Motor to control the angle of the wheel
    private final RelativeEncoder driveEncoder;  // Encoder to measure speed
    private final RelativeEncoder steeringEncoder;  // Encoder to measure the wheel's angle

    private final Pigeon2 pigeonGyro;  // Pigeon2 Gyro for overall robot heading

    private final NetworkTable moduleTable;  // NetworkTable for reporting module's data

    // Constructor that takes drive motor, steering motor, encoders, and pigeon
    public SwerveModule(int driveMotorPort, int steeringMotorPort, int driveEncoderPort, int steeringEncoderPort, Pigeon2 pigeonGyro, String moduleName) {
        this.driveMotor = new SparkMax(driveMotorPort, MotorType.kBrushless);  // Initialize drive motor
        this.steeringMotor = new SparkMax(steeringMotorPort, MotorType.kBrushless);  // Initialize steering motor
        this.driveEncoder = driveMotor.getEncoder();  // Initialize drive encoder
        this.steeringEncoder = steeringMotor.getEncoder();  // Initialize steering encoder

        this.pigeonGyro = pigeonGyro;  // Initialize the Pigeon2 Gyro

        // Initialize a module-specific NetworkTable for reporting
        this.moduleTable = NetworkTableInstance.getDefault().getTable("SwerveModules").getSubTable(moduleName);
    }

    // Set the desired state (angle and speed) of the swerve module
    public void setDesiredState(double forward, double strafe, double rotation) {
        // Calculate desired angle based on forward and strafe values
        double desiredAngle = Math.atan2(forward, strafe);  // This is the desired steering angle
        double desiredSpeed = Math.sqrt(forward * forward + strafe * strafe);  // Desired speed is based on forward and strafe components

        // Set steering motor to the calculated angle
        steeringMotor.set(desiredAngle);  // In this example, the steering motor is directly set to the desired angle

        // Set drive motor speed to the desired speed
        driveMotor.set(desiredSpeed);  // In this example, the speed is directly set to the desired speed
    }

    // Get the current velocity of the swerve module
    public Translation2d getVelocity() {
        double speed = driveEncoder.getVelocity();  // Get the speed from the drive encoder (in units per second)
        double angle = steeringEncoder.getPosition();  // Get the angle from the steering encoder (in degrees)

        // Convert the speed and angle to Cartesian velocity components using Translation2d
        double velocityX = speed * Math.cos(Math.toRadians(angle));  // Calculate the X component of the velocity
        double velocityY = speed * Math.sin(Math.toRadians(angle));  // Calculate the Y component of the velocity

        return new Translation2d(velocityX, velocityY);  // Return the velocity as a Translation2d
    }

    // Update the NetworkTable with the module's current angle and velocity
    public void updateNetworkTable() {
        // Get the current angle of the module from its steering encoder
        double currentAngle = steeringEncoder.getPosition();  // Angle in degrees
        Translation2d currentVelocity = getVelocity();  // Get the velocity vector (X, Y)

        // Report to NetworkTable: module's angle and velocity
        moduleTable.getEntry("angle").setDouble(currentAngle);  // Set the angle of the module
        moduleTable.getEntry("velocityX").setDouble(currentVelocity.getX());  // Set the X component of velocity
        moduleTable.getEntry("velocityY").setDouble(currentVelocity.getY());  // Set the Y component of velocity
    }

    public double getAngle(){
        return steeringEncoder.getPosition();  // Get the current angle of the moduleß
    }
}
