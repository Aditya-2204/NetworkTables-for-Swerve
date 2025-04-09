package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;
  private final NetworkTable swerveTable;

  // Define the robot’s swerve geometry (module positions relative to center, in meters)
  private final SwerveDriveKinematics kinematics;

  public SwerveDrive() {
    this.swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");

    Translation2d frontLeftLocation  = new Translation2d(0.25,  0.25);
    Translation2d frontRightLocation = new Translation2d(0.25, -0.25);
    Translation2d backLeftLocation   = new Translation2d(-0.25,  0.25);
    Translation2d backRightLocation  = new Translation2d(-0.25, -0.25);

    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    // Instantiate each swerve module with example CAN IDs (update these IDs as needed).
    frontLeft  = new SwerveModule("frontLeft", 1, 2);
    frontRight = new SwerveModule("frontRight", 3, 4);
    backLeft   = new SwerveModule("backLeft", 5, 6);
    backRight  = new SwerveModule("backRight", 7, 8);
  }

  /**
   * Drive the robot using actual swerve kinematics.
   *
   * @param forward  Forward speed in m/s.
   * @param strafe   Left/right speed in m/s.
   * @param rotation Rotational speed in rad/s.
   */
  public void drive(double forward, double strafe, double rotation) {
    // Create a ChassisSpeeds object from the desired speeds.
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forward, strafe, rotation);

    // Compute desired states for each swerve module.
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // Optionally desaturate wheel speeds so that no module exceeds maximum velocity.
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModule.MAX_VELOCITY);

    // Command each module with its computed state.
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);

    // Publish each module's angle and velocity to NetworkTables.

  }
  @Override
  public void periodic() {
    // Update the NetworkTable with module data.
    frontLeft.updateNetworkTable(swerveTable);
    frontRight.updateNetworkTable(swerveTable);
    backLeft.updateNetworkTable(swerveTable);
    backRight.updateNetworkTable(swerveTable);
  }
}
