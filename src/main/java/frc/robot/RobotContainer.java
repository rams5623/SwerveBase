// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final Trigger xboxA = driverXbox.a();
  private final Trigger xboxX = driverXbox.x();
  private final Trigger xboxLT = driverXbox.leftTrigger(0.2);
  private final Trigger xboxRT = driverXbox.rightTrigger(0.2);






  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -0.9,
                                                                () -> driverXbox.getLeftX() * -0.9)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -0.9)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);

SwerveInputStream driveAngularVelocity_Slow = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                               () -> driverXbox.getLeftY() * -0.35,
                                                               () -> driverXbox.getLeftX() * -0.35)
                                                           .withControllerRotationAxis(() -> driverXbox.getRightX() * -.5)
                                                           .deadband(OperatorConstants.DEADBAND)
                                                           .scaleTranslation(0.8)
                                                           .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
  //                                                                                            driverXbox::getRightY)
  //                                                          .headingWhile(true);


  // SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //                                                                  () -> -driverXbox.getLeftY(),
  //                                                                  () -> -driverXbox.getLeftX())
  //                                                              .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
  //                                                              .deadband(OperatorConstants.DEADBAND)
  //                                                              .scaleTranslation(0.8)
  //                                                              .allianceRelativeControl(true);
  // // Derive the heading axis with math!
  // SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
  //                                                                    .withControllerHeadingAxis(() -> Math.sin(
  //                                                                                                   driverXbox.getRawAxis(
  //                                                                                                       2) * Math.PI) * (Math.PI * 2),
  //                                                                                               () -> Math.cos(
  //                                                                                                   driverXbox.getRawAxis(
  //                                                                                                       2) * Math.PI) *
  //                                                                                                     (Math.PI * 2))
  //                                                                    .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    // Command driveFieldOrientedDirectAngle         = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocity_Slow    = drivebase.driveFieldOriented(driveAngularVelocity_Slow);
    // Command driveSetpointGen                      = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    // Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
    // Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    // Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
    //     driveDirectAngleSim);

    // SET THE DRIVE TYPE
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      /*
       * A = Zero Gyro
       * X = Fake Vision Reading
       * B = DRIVE TO POSITION
       * Y = Slow Mode
       * START = No Command
       * BACK = No Command
       * Left Bump = Lock Wheels
       * Right Bump = No Command
       */
      xboxA
        .onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );
      // driverXbox.start().whileTrue(Commands.none());
      // driverXbox.back().whileTrue(Commands.none());
     xboxX
      .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    xboxLT
      .or(xboxRT)
      .whileTrue(driveFieldOrientedAnglularVelocity_Slow);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
