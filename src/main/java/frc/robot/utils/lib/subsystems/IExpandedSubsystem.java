// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib.subsystems;

public interface IExpandedSubsystem {
    default public void earlyPeriodic() {}

    default public void latePeriodic() {}
}
