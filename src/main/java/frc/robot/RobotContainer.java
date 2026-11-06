// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.PosePhoton;
import frc.robot.subsystems.Swerve;
import frc.robot.Telemetry;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final static Swerve drivetrain = Constants.OperatorConstants.createDrivetrain();
  private final Telemetry logger = new Telemetry(20);
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public PosePhoton vision = new PosePhoton();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
            drivetrain.setDefaultCommand(  
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                // raw desired velocities from joystick

                
                double desiredX = -applyDeadzone(driver.getRawAxis(Constants.AXIS_Y), 0.02) * MaxSpeed * reverse; // forward
                double desiredY = -applyDeadzone(driver.getRawAxis(Constants.AXIS_X), 0.02) * MaxSpeed * reverse; // left
                double desiredOmega = -applyDeadzone(driver.getRawAxis(Constants.AXIS_TWIST), 0.02) * MaxAngularRate; // rotate

                double PROGdesiredX = -applyDeadzone(progJoystick.getRawAxis(Constants.AXIS_Y), 0.02) * MaxSpeed * reverse; // forward
                double PROGdesiredY = -applyDeadzone(progJoystick.getRawAxis(Constants.AXIS_X), 0.02) * MaxSpeed * reverse; // left
                double PROGdesiredOmega = -applyDeadzone(progJoystick.getRawAxis(Constants.AXIS_TWIST), 0.02) * MaxAngularRate; // rotate
                // reverse = SmartDashboard.getNumber("Reverse", reverse);
                if (SmartDashboard.getString("Alliance", "Red").equals("Blue")){
                    reverse = -1;
                } else {
                    reverse = 1;
                }
                SmartDashboard.putNumber("DesiredX", PROGdesiredX);
                SmartDashboard.putNumber("DesiredY", PROGdesiredY);
                SmartDashboard.putNumber("DesiredOmega", PROGdesiredOmega);

                // double desiredX = -applyDeadzone(driver.getRawAxis(Constants.AXIS_Y), 0.2) * MaxSpeed; // forward
                // double desiredY = -applyDeadzone(driver.getRawAxis(Constants.AXIS_X), 0.2) * MaxSpeed; // left
                // double desiredOmega = -applyDeadzone(driver.getRawAxis(Constants.AXIS_TWIST), 0.2) * MaxAngularRate; // rotate

                // Read Shuffleboard tunables and apply them to the limiter
                try {
                    double rx = m_rateXEntry.getDouble(2.0);
                    double ry = m_rateYEntry.getDouble(2.0);
                    double ro = m_rateOmegaEntry.getDouble(4.0);
                    m_trapezoidLimiter.setRates(rx, ry, ro);
                } catch (Throwable t) {
                    // ignore and use current rates
                }
                // Apply trapezoidal limiter (slightly steep ramp)
                double[] smoothed = m_trapezoidLimiter.calculate(desiredX, desiredY, desiredOmega);
                if (PROGdesiredX != 0 || PROGdesiredY != 0 || PROGdesiredOmega != 0) {
                    Constants.TurretConstants.antiRotationOffset = Constants.TurretConstants.antiMultiplier * PROGdesiredOmega;
                    SmartDashboard.putNumber("antiRotation", Constants.TurretConstants.antiRotationOffset);
                    return drive
                    .withVelocityX(PROGdesiredX)
                    .withVelocityY(PROGdesiredY)
                    .withRotationalRate(PROGdesiredOmega);
                } else {
                    Constants.TurretConstants.antiRotationOffset = Constants.TurretConstants.antiMultiplier * desiredOmega;
                    SmartDashboard.putNumber("antiRotation", Constants.TurretConstants.antiRotationOffset);
                    return drive
                    .withVelocityX(smoothed[0])
                    .withVelocityY(smoothed[1])
                    .withRotationalRate(smoothed[2]);}
            }));
    
            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.
            final var idle = new SwerveRequest.Idle();
            RobotModeTriggers.disabled().whileTrue(
                    drivetrain.applyRequest(() -> idle).ignoringDisable(true));
          
            // zeroGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

            // slowMode.onTrue(drivetrain.runOnce(() -> {
            //     MaxSpeed = normalMaxSpeed * kSlowMultiplier;
            //     MaxAngularRate = normalMaxAngularRate * kSlowMultiplier;
            // }));
            // slowMode.onFalse(drivetrain.runOnce(() -> {
            //     MaxSpeed = normalMaxSpeed;
            //     MaxAngularRate = normalMaxAngularRate;
            // }));

            PROGzeroGyro.onTrue(drivetrain.runOnce(() -> {
                drivetrain.seedFieldCentric();
                Robot.requestVisionReseed();
            }));

            // slow mode (hold button 6 to reduce speeds)
            PROGslowMode.onTrue(drivetrain.runOnce(() -> {
                MaxSpeed = normalMaxSpeed * (kSlowMultiplier*1.4);
                MaxAngularRate = normalMaxAngularRate * (kSlowMultiplier+0.2);
            }));
            PROGslowMode.onFalse(drivetrain.runOnce(() -> {
                MaxSpeed = normalMaxSpeed;
                MaxAngularRate = normalMaxAngularRate;
            }));
    
            drivetrain.registerTelemetry(logger::telemeterize);
        }
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
