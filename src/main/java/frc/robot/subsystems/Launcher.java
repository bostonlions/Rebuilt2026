package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Robot.Ports;

import static frc.robot.Robot.kCANBus;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

public final class Launcher implements Subsystem {
    private static Launcher instance = null;
    private final Slot0Configs gains = new Slot0Configs()
        .withKP(4.8).withKD(0.1).withKV(0.12).withKS(0.2).withKA(0.01); // TODO: tune this, maybe different for each motor
    private final TalonFX launchMotor = new TalonFX(Ports.LAUNCHER, kCANBus);
    private final CANcoder launchCAN = new CANcoder(Ports.LAUNCH_CANCODER, kCANBus);
    private final TalonFX pitchMotor = new TalonFX(Ports.PITCH_MOTOR, kCANBus);
    private final CANcoder pitchCAN = new CANcoder(Ports.PITCH_CANCODER, kCANBus);
    private final TalonFX yawMotor = new TalonFX(Ports.YAW_MOTOR, kCANBus);
    private final CANcoder yawCAN = new CANcoder(Ports.YAW_CANCODER, kCANBus);
    
    public static Launcher getInstance() {
        if (instance == null) instance = new Launcher();
        return instance;
    }

    private Launcher() {
        launchMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(gains)
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8)))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Amps.of(40)).withPeakReverseTorqueCurrent(Amps.of(-40)))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withFeedbackRemoteSensorID(launchCAN.getDeviceID())));
        pitchMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(gains)
            .withFeedback(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withFeedbackRemoteSensorID(pitchCAN.getDeviceID())));
        yawMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withSlot0(gains)
            .withFeedback(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withFeedbackRemoteSensorID(yawCAN.getDeviceID())));
    }

    public void setSpeed(double rps) {
        launchMotor.setControl(new VelocityVoltage(rps));
    }

    public void setPitch(double degrees) {
        pitchMotor.setPosition(degrees / 360);
    }

    public void setYaw(double degrees) {
        yawMotor.setPosition(degrees / 360);
    }
}
