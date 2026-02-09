// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ControlBoard.CustomXboxController.Button;

public final class RobotContainer {
    private final CANrange canRange = new CANrange(Constants.Ports.CANRANGE, Constants.SwerveConstants.kCANBus);
    private final Telemetry logger = new Telemetry(Constants.SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond));
    private final ControlBoard controller = ControlBoard.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    public RobotContainer() {
        Robot.pigeon.getConfigurator().apply(new Pigeon2Configuration());
        canRange.getConfigurator().apply(new CANrangeConfiguration());

        new Trigger(() -> controller.operator.getButton(Button.LB)).whileTrue(
            new InstantCommand(() -> Constants.LauncherConstants.motor.setControl(Constants.LauncherConstants.velocityVoltage))
        ).whileFalse(new InstantCommand(() -> Constants.LauncherConstants.motor.setControl(Constants.LauncherConstants.brake)));

        drivetrain.setDefaultCommand( // X is defined as forward according to WPILib convention, and Y is defined as to the left
            drivetrain.applyRequest(() -> // ...but our controller convention is y as forward, and x as to the right
                Constants.SwerveConstants.drive.withVelocityX(controller.getSwerveTranslation().getY())
                    .withVelocityY(-controller.getSwerveTranslation().getX())
                    .withRotationalRate(-controller.getSwerveRotation())
            )
        );

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> Constants.SwerveConstants.idle).ignoringDisable(true)
        );

        new Trigger(() -> controller.operator.getButton(Button.RB)).onTrue(
            new InstantCommand(() -> Robot.pigeon.setYaw(0)).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);

        new Trigger(() -> controller.operator.getButton(Button.B)).onTrue(new InstantCommand(() -> {
            System.out.println("CANrange distance: " + canRange.getDistance().getValueAsDouble());
            
            var pose = LimelightHelpers.getRobotPose();
            System.out.println(pose[0] + ", " + pose[1] + ", " + pose[2] + "; " + pose[6]);
        }));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton:
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                Constants.SwerveConstants.drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> Constants.SwerveConstants.idle)
        );
    }
}
