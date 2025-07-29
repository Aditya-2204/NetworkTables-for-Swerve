package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

=======
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
>>>>>>> ac3007a70e0f9c5d4069021329f50620d11d7884

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule leftFront;
  private final SwerveModule rightFront;
  private final SwerveModule leftBack;
  private final SwerveModule rightBack;
  private final NetworkTable swerveTable;
  private Pigeon2 gyro;
  private RobotConfig config;
<<<<<<< HEAD
  private final Field2d field = new Field2d();



=======
  private NetworkTableInstance inst;
  private Field2d field;
>>>>>>> ac3007a70e0f9c5d4069021329f50620d11d7884

  // Define the robot’s swerve geometry (module positions relative to center, in meters)
  private final SwerveDriveKinematics kinematics;

  private final SwerveDrivePoseEstimator poseEstimator;
  private NetworkTableInstance Table;
  DataLog log;
  
      
    
  public SwerveDrive() {
    DataLogManager.start();
    // Set up custom log entries
    log = DataLogManager.getLog();



    field = new Field2d();
    this.Table = NetworkTableInstance.getDefault();
    this.Table.startServer();
    this.swerveTable = Table.getTable("SwerveDrive");

    Translation2d leftFrontLocation  = new Translation2d(0.25,  0.25);
    Translation2d rightFrontLocation = new Translation2d(0.25, -0.25);
    Translation2d leftBackLocation   = new Translation2d(-0.25,  0.25);
    Translation2d rightBackLocation  = new Translation2d(-0.25, -0.25);

    kinematics = new SwerveDriveKinematics(
        leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation);

    // Instantiate each swerve module with example CAN IDs (update these IDs as needed).
    leftFront  = new SwerveModule("leftFront", 1, 2);
    rightFront = new SwerveModule("rightFront", 3, 4);
    leftBack   = new SwerveModule("leftBack", 5, 6);
    rightBack  = new SwerveModule("rightBack", 7, 8);

    gyro = new Pigeon2(SwerveConstants.PIGEON_ID);

    poseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.DRIVE_KINEMATICS,
      getHeadingRotation2d(),
      getModulePositions(),
      new Pose2d()
    );

      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }
  
      // Configure AutoBuilder last
      AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(1.0, 3.0, 4.0), // Translation PID constants
                      new PIDConstants(2.0, 5.0, 8.0) // Rotation PID constants
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );

    SmartDashboard.putData("Field", field);
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
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);

    // Publish each module's angle and velocity to NetworkTables.

  }
  @Override
  public void periodic() {

    // Update the Field2d with the current estimated pose
    field.setRobotPose(getPose());

    // Update the NetworkTable with module data.
    leftFront.updateNetworkTable(log);
    rightFront.updateNetworkTable(log);
    leftBack.updateNetworkTable(log);
    rightBack.updateNetworkTable(log);



    //Update pose estimator
    poseEstimator.update(getHeadingRotation2d(), getModulePositions());

    if(RobotState.isAutonomous()) swerveTable.getEntry("Robot State").setString("Autonomous");
    else if(RobotState.isTeleop()) swerveTable.getEntry("Robot State").setString("Teleop");
    else if(RobotState.isDisabled()) swerveTable.getEntry("Robot State").setString("Disabled");
    else if(RobotState.isTest()) swerveTable.getEntry("Robot State").setString("Test");
    else swerveTable.getEntry("Robot State").setString("Unknown");
    

    System.out.println(poseEstimator.getEstimatedPosition().getX() + ", " + poseEstimator.getEstimatedPosition().getY() + ", " + poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }
  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getHeading(){
    return Math.IEEEremainder(-gyro.getYaw().getValueAsDouble(), 360); //clamp heading between -180 and 180
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = leftFront.getState();
    states[1] = rightFront.getState();
    states[2] = leftBack.getState();
    states[3] = rightBack.getState();
    return states;
  } 

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
  }
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }
}
