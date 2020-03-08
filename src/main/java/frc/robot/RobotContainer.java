
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Xbox;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SubsistemaDrivetrain;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static SubsistemaDrivetrain Swerve = new SubsistemaDrivetrain();
  public static SwerveDrive ComandoSwerve = new SwerveDrive();

  // private final ExampleCommand autoCommand = new ExampleCommand(Swervemodule);

  public static XboxController Contorl = new XboxController(Xbox.Control1X);




  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }


  /* 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
  */
}
