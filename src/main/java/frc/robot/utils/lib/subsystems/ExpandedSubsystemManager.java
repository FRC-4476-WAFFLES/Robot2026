// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib.subsystems;

import java.util.ArrayList;

public class ExpandedSubsystemManager {
    private static ArrayList<IExpandedSubsystem> subsystemList = new ArrayList<>();
    private static ArrayList<VirtualSubsystem> virtualSubsystemList = new ArrayList<>();

    public static void RegisterSubsystem(IExpandedSubsystem system) {
        if (subsystemList.contains(system)) {
            System.out.println("Cannot register same subsystem twice. Class was: " + system.getClass().getName());
            return;
        }

        subsystemList.add(system);
    }

    public static void RegisterVirtualSubsystem(VirtualSubsystem system) {
        if (virtualSubsystemList.contains(system)) {
            System.out.println("Cannot register same subsystem twice. Class was: " + system.getClass().getName());
            return;
        }

        virtualSubsystemList.add(system);
    }

    public static void RunEarlyPeriodic() {
        for (IExpandedSubsystem subsystem : subsystemList) {
            subsystem.earlyPeriodic();
        }
    }

    public static void RunLatePeriodic() {
        // Run for virtual subsystems first
        for (VirtualSubsystem subsystem : virtualSubsystemList) {
            subsystem.periodic();
        }
        for (IExpandedSubsystem subsystem : subsystemList) {
            subsystem.latePeriodic();
        }
    }
}
