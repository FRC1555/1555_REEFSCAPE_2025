// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.event.BooleanEvent;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

  // The driver's controller
  public Joystick m_driverController =
      new Joystick(OIConstants.kDriverControllerPort);
  public CommandXboxController m_manipController =
      new CommandXboxController(OIConstants.kManipControllerPort);

  // Joystick buttons for speed control
  public JoystickButton fullSendButton = new JoystickButton(m_driverController, 1);
  public JoystickButton highSpeedButton = new JoystickButton(m_driverController, 2);
  public JoystickButton midSpeedButton = new JoystickButton(m_driverController, 3);
  public JoystickButton lowSpeedButton = new JoystickButton(m_driverController, 4);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //registering named Commands for Algae
    NamedCommands.registerCommand("Grab Algae", m_algaeSubsystem.runIntakeCommand());
    NamedCommands.registerCommand("Spit Out Algae", m_algaeSubsystem.reverseIntakeCommand());
    //registering named commands for Coral
    NamedCommands.registerCommand("Grab Coral", m_coralSubSystem.runIntakeCommand());
    NamedCommands.registerCommand("Spit Out Coral", m_coralSubSystem.reverseIntakeCommand());
    NamedCommands.registerCommand("Coral Station", m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation));
    NamedCommands.registerCommand("L2", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
    NamedCommands.registerCommand("L3", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    NamedCommands.registerCommand("L4", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));
    //building the auto chooser on smartdashboard
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getZ(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));

    // Set the ball intake to in/out when not running based on internal state
    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {



    // Left Bumper -> Run tube intake
    m_manipController.rightBumper().whileTrue(m_coralSubSystem.runIntakeCommand());

    // Right Bumper -> Run tube intake in reverse
    m_manipController.leftBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());

    // B Button -> Elevator/Arm to human player position, set ball intake to stow
    // when idle
    m_manipController
        .b()
        .onTrue(
            m_coralSubSystem
                .setSetpointCommand(Setpoint.kFeederStation)
                .alongWith(m_algaeSubsystem.stowCommand()));

    // A Button -> Elevator/Arm to level 2 position
    m_manipController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));

    // X Button -> Elevator/Arm to level 3 position
    m_manipController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));

    // Y Button -> Elevator/Arm to level 4 position
    m_manipController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));

    // Right Trigger -> Run ball intake, set to leave out when idle
    m_manipController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.runIntakeCommand());

    // Left Trigger -> Run ball intake in reverse, set to stow when idle
    m_manipController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.reverseIntakeCommand());

    // Turtle Beach Button 1 or Colored A -> Set Robot Speed to Full Send (Child Detected)
    fullSendButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(1)));
    highSpeedButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(0.75)));
    midSpeedButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(0.5)));
    lowSpeedButton.onTrue(new InstantCommand(() -> m_robotDrive.setDriveSpeed(0.25)));

        // Bind buttons 5, 6, 7, and 8 to reset the gyro
        new JoystickButton(m_driverController, 5)
            .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
        new JoystickButton(m_driverController, 6)
            .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
        new JoystickButton(m_driverController, 7)
            .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));
        new JoystickButton(m_driverController, 8)
            .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));



  }

  public double getSimulationTotalCurrentDraw() {
    // for each subsystem with simulation
    return m_coralSubSystem.getSimulationCurrentDraw()
        + m_algaeSubsystem.getSimulationCurrentDraw();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private final SendableChooser<Command> autoChooser;
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}