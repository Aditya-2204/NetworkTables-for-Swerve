package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private Command autonomousCommand;

  @Override
  public void robotInit() {
    // Instantiate RobotContainer which sets up subsystems and commands.
    robotContainer = new RobotContainer();
  }

  @Override
  public void autonomousInit() {
    // Retrieve and schedule the autonomous command from RobotContainer.
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Run the command scheduler to execute the autonomous command.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // Cancel autonomous command when teleop starts.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Run the command scheduler in teleop mode.
    CommandScheduler.getInstance().run();
  }
}
