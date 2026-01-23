package frc.robot.subsystems.superstructure.shooter;
import frc.robot.utils.hardware.TalonFXIO.TalonFXIOData;
public interface ShooterIO {
  class ShooterIOInputs {
        public TalonFXIOData shooterMotorData = new TalonFXIOData(0, 0, 0, 0, 0, 0, 0, 0);
  }
  default void updateInputs(ShooterIOInputs inputs) {}
  
  default void runDutyCycle(double speed) {}

  default void runShooterVelocity(double velocity) {}
}
