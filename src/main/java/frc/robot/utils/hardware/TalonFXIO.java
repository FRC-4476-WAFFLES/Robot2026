// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Ports;

/** A shim on top of talonFX motors which optimizes their CAN usage automatically */
public class TalonFXIO extends TalonFX {
  // A collection of signals needed by robot code
  public record TalonFXIOSignals(
      StatusSignal<Angle> position,
      StatusSignal<AngularVelocity> velocity,
      StatusSignal<Voltage> motorVoltage,
      StatusSignal<Current> torqueCurrent,
      StatusSignal<Current> supplyCurrent,
      StatusSignal<Current> statorCurrent,
      StatusSignal<Double> dutyCycle,
      StatusSignal<Temperature> temp
  ) {}

  // Serializable data
  public record TalonFXIOData(
      double position,
      double velocity,
      double motorVoltage,
      double torqueCurrent,
      double supplyCurrent,
      double statorCurrent,
      double dutyCycle,
      double temp
  ) {}

  private final boolean isCANFD;
  private final String CANName;

  private TalonFXIOSignals statusSignals;

  /**
   * Constructs a TalonFXIO from a port, which carries both the CAN ID and the bus.
   * Prefer this over the raw ID constructors.
   */
  public TalonFXIO(Ports port) {
    super(port.id, port.bus);

    isCANFD = port.isCANFD();
    CANName = port.bus.getName();
    setup();
  }

  /**
   * Constructs a TalonFXIO with a CAN ID
   */
  public TalonFXIO(int ID) {
    super(ID);

    isCANFD = false;
    CANName = "rio";
    setup();
  }

  /**
   * Constructs a TalonFXIO with a CAN ID and Canbus
   */
  public TalonFXIO(int ID, CANBus Canbus) {
    super(ID, Canbus);

    isCANFD = Canbus.isNetworkFD();
    CANName = Canbus.getName();
    setup();
  }

  /*
   * Initializes all status signals
   */
  private void setup() {
    // Init record with signals  
    statusSignals = new TalonFXIOSignals(
        getPosition(),
        getVelocity(),
        getMotorVoltage(),
        getTorqueCurrent(),
        getSupplyCurrent(),
        getStatorCurrent(),
        getDutyCycle(),
        getDeviceTemp()
    );

    // Set update rate for used signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        isCANFD ? CodeConstants.FD_CAN_FREQUENCY : CodeConstants.BASE_CAN_FREQUENCY,
        statusSignals.position,
        statusSignals.velocity,
        statusSignals.motorVoltage,
        statusSignals.torqueCurrent,
        statusSignals.supplyCurrent,
        statusSignals.statorCurrent,
        statusSignals.dutyCycle,
        statusSignals.temp
    );

    // Eliminate unused signals
    optimizeBusUtilization(CodeConstants.DISABLE_UNUSED_STATUS_SIGNALS ? 0 : 4, 0.1);

    // Register signals to be refreshed as a group
    PhoenixHelpers.RegisterStatusSignals(
        CANName,
        statusSignals.position,
        statusSignals.velocity,
        statusSignals.motorVoltage,
        statusSignals.torqueCurrent,
        statusSignals.supplyCurrent,
        statusSignals.statorCurrent,
        statusSignals.dutyCycle,
        statusSignals.temp
    );
  }

  // In case timing data is desired or something
  public TalonFXIOSignals getRawSignals() {
    return statusSignals;
  }

  public TalonFXIOData getSignalData() {
    return new TalonFXIOData(
        statusSignals.position.getValueAsDouble(),
        statusSignals.velocity.getValueAsDouble(),
        statusSignals.motorVoltage.getValueAsDouble(),
        statusSignals.torqueCurrent.getValueAsDouble(),
        statusSignals.supplyCurrent.getValueAsDouble(),
        statusSignals.statorCurrent.getValueAsDouble(),
        statusSignals.dutyCycle.getValueAsDouble(),
        statusSignals.temp.getValueAsDouble()
    );
  }
}
