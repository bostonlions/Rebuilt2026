// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.HootAutoReplay;

public class Robot extends TimedRobot {
    public static final Pigeon2 pigeon = new Pigeon2(Constants.Ports.PIGEON, Constants.Ports.CANBUS_DRIVE);
    private final CANrange canRange = new CANrange(Constants.Ports.CANRANGE, Constants.Ports.CANBUS_DRIVE);
    private final RobotContainer m_robotContainer = new RobotContainer();
    private Command m_autonomousCommand;
    // Log and replay timestamp and joystick data:
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay().withTimestampReplay().withJoystickReplay();

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        canRange.getConfigurator().apply(new CANrangeConfiguration());
    }

    @Override
    public void disabledPeriodic() {
        LimelightHelpers.SetRobotOrientation(
            "limelight",
            Rotation2d.fromDegrees(
                Rotation2d.fromDegrees(pigeon.getYaw().getValue().in(Units.Degrees)).rotateBy(new Rotation2d().unaryMinus()).getDegrees()
            ).getDegrees(), 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetIMUMode("limelight", 1);
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
            "limelight",
            Rotation2d.fromDegrees(
                Rotation2d.fromDegrees(pigeon.getYaw().getValue().in(Units.Degrees)).rotateBy(new Rotation2d().unaryMinus()).getDegrees()
            ).getDegrees(), 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetIMUMode("limelight", 4);
        double[] visMeasurement = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);

        if (MathUtil.isNear(500, System.currentTimeMillis() % 1000, 20)) {
            System.out.println("X: " + visMeasurement[0] + "; Y: " + visMeasurement[1] + "; Z: " + visMeasurement[2] +
                "; Roll: " + visMeasurement[3] + "; Pitch: " + visMeasurement[4] + "; Yaw: " + visMeasurement[5]);
            System.out.println("CANrange distance: " + canRange.getDistance().getValueAsDouble());
        }
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
