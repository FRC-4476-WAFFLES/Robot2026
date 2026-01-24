// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import frc.robot.data.Constants;
import frc.robot.utils.hardware.PhoenixHelpers;
import frc.robot.utils.hardware.TalonFXIO;

public class ShooterIOTalonFX implements ShooterIO {
    // Hardware Components
    protected final TalonFXIO shooter;

    // Control Objects
    private final MotionMagicVelocityVoltage shooterVelocityRequest = new MotionMagicVelocityVoltage(0);

    public ShooterIOTalonFX() {
      shooter = new TalonFXIO(Constants.CANIds.shooterMotor);
        // Configure hardware
        configureShooterMotor();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      inputs.shooterMotorData = shooter.getSignalData();
    }

    @Override
    public void runDutyCycle(double speed) {
        shooter.set(speed);
    }

    @Override
    public void runShooterVelocity(double velocity) {
      shooter.setControl(shooterVelocityRequest.withVelocity(velocity));
    }

    /**
    * Configures the shooter motor with current limits
    */
    private void configureShooterMotor() {
        TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
        CurrentLimitsConfigs shooterCurrentLimit = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true);

        shooterConfigs.CurrentLimits = shooterCurrentLimit;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 1;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 1.5;
        slot0Configs.kG = 0.0;
        shooterConfigs.Slot0 = slot0Configs;

        // Motion Magic
        MotionMagicConfigs motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicAcceleration = 200;
        motionMagic.MotionMagicJerk = 0;
        shooterConfigs.MotionMagic = motionMagic;

        PhoenixHelpers.tryConfig(() -> shooter.getConfigurator().apply(shooterConfigs));
    }
}
