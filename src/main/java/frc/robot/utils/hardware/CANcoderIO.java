// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** A shim on top of CANcoders which optimizes their CAN usage automatically */
public class CANcoderIO extends CANcoder {
    // A collection of signals needed by robot code
    public record CANcoderIOSignals(
            StatusSignal<Angle> position,
            StatusSignal<AngularVelocity> velocity,
            StatusSignal<Angle> absolutePosition) {
    }

    private final String CANName;

    private CANcoderIOSignals statusSignals;

    /**
     * Constructs a CANcoderIO with a CAN ID
     */
    public CANcoderIO(int ID) {
        super(ID);

        CANName = "rio";
        setup();
    }

    /**
     * Constructs a CANcoderIO with a CAN ID and Canbus
     */
    public CANcoderIO(int ID, CANBus Canbus) {
        super(ID, Canbus);

        CANName = Canbus.getName();
        setup();
    }

    /*
     * Initializes all status signals
     */
    private void setup() {
        // Init record with signals  
        statusSignals = new CANcoderIOSignals(
                getPosition(),
                getVelocity(),
                getAbsolutePosition()
        );

        // Set update rate for used signals
        BaseStatusSignal.setUpdateFrequencyForAll(
                100, // Always use 100hz
                statusSignals.position,
                statusSignals.velocity,
                statusSignals.absolutePosition
        );

        // Eliminate unused signals
        // optimizeBusUtilization(CodeConstants.DISABLE_UNUSED_STATUS_SIGNALS ? 0 : 4, 0.2);

        // Register signals to be refreshed as a group
        PhoenixHelpers.RegisterStatusSignals(
                CANName,
                statusSignals.position,
                statusSignals.velocity,
                statusSignals.absolutePosition
        );
    }

    public CANcoderIOSignals signals() {
        return statusSignals;
    }
}
