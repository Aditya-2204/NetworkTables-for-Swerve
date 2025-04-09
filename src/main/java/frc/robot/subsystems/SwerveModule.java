package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;

public class SwerveModule {
  // Define the maximum velocity for scaling (in meters per second).
  public static final double MAX_VELOCITY = 5.0;

  private final String name;
  private final SparkMax driveMotor;
  private final SparkMax turningMotor;
  private SwerveModuleState currentState;

  public SwerveModule(String name, int driveMotorID, int turningMotorID) {
    this.name = name;
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);
    // Initialize state to zero.
    currentState = new SwerveModuleState(0.0, new Rotation2d(0.0));
  }

  /**
   * Sets the desired state using real swerve kinematics.
   * In a full implementation, you’d incorporate PID control for both drive and turning.
   *
   * @param state The desired SwerveModuleState from kinematics.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Optimize the module state to reduce unnecessary rotation.
    currentState = SwerveModuleState.optimize(state, new Rotation2d(getCurrentAngleRadians()));

    // Calculate drive motor output (scaled to -1 to 1 based on MAX_VELOCITY).
    double driveOutput = currentState.speedMetersPerSecond / MAX_VELOCITY;
    driveMotor.set(driveOutput);

    // For the turning motor, convert the desired angle to degrees.
    double targetAngleDegrees = currentState.angle.getDegrees();
    // In a complete implementation, use a PID controller to reach targetAngleDegrees.
    // Here we simply set the turning motor’s encoder position.
    turningMotor.getEncoder().setPosition(targetAngleDegrees);
  }

  /**
   * Retrieves the current angle of the module in radians.
   * This dummy conversion assumes the turning encoder outputs degrees.
   */
  private double getCurrentAngleRadians() {
    return Math.toRadians(turningMotor.getEncoder().getPosition());
  }

  /**
   * Reports the module’s current angle (in degrees) and velocity (m/s) to NetworkTables.
   *
   * @param table The NetworkTable where the values will be published.
   */
  public void updateNetworkTable(NetworkTable table) {
    table.getEntry(name + "/angle").setDouble(currentState.angle.getDegrees());
    table.getEntry(name + "/velocity").setDouble(currentState.speedMetersPerSecond);
  }
}
