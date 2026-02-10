// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.HootAutoReplay;

public class Robot extends TimedRobot {
    public static final Pigeon2 pigeon = new Pigeon2(Constants.Ports.PIGEON, Constants.Ports.CANBUS_DRIVE);
    private final RobotContainer m_robotContainer = new RobotContainer();
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay().withTimestampReplay().withJoystickReplay();
    private Command m_autonomousCommand;

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        var pose = LimelightHelpers.getRobotPose();
        m_robotContainer.drivetrain.addVisionMeasurement(new Pose2d(pose[0], pose[1], new Rotation2d(pose[5])), (
            NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("ts_nt").getDouble(-1) +
            NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("ts_nt").getDouble(-1)
        ) / 2000000, MatBuilder.fill(Nat.N3(), Nat.N1(), pose[6], pose[6], 0.001));
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-a",
            Rotation2d.fromDegrees(
                Rotation2d.fromDegrees(pigeon.getYaw().getValue().in(Units.Degrees)).rotateBy(new Rotation2d().unaryMinus()).getDegrees()
            ).getDegrees(), 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetIMUMode("limelight-a", 1);
        LimelightHelpers.SetRobotOrientation(
            "limelight-b",
            Rotation2d.fromDegrees(
                Rotation2d.fromDegrees(pigeon.getYaw().getValue().in(Units.Degrees)).rotateBy(new Rotation2d().unaryMinus()).getDegrees()
            ).getDegrees(), 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetIMUMode("limelight-b", 1);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }

    @Override
    public void teleopPeriodic() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-a",
            Rotation2d.fromDegrees(
                Rotation2d.fromDegrees(pigeon.getYaw().getValue().in(Units.Degrees)).rotateBy(new Rotation2d().unaryMinus()).getDegrees()
            ).getDegrees(), 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetIMUMode("limelight-a", 4);
        LimelightHelpers.SetRobotOrientation(
            "limelight-b",
            Rotation2d.fromDegrees(
                Rotation2d.fromDegrees(pigeon.getYaw().getValue().in(Units.Degrees)).rotateBy(new Rotation2d().unaryMinus()).getDegrees()
            ).getDegrees(), 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetIMUMode("limelight-b", 4);
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
