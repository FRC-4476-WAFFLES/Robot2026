package frc.robot.utils.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public class SecondOrderSim {
  private double k1, k2, k3;
  private double previousnput;
  private double position;
  private double velocity;

  public SecondOrderSim(double frequency, double damping, double response, double startingPosition) {
    previousnput = startingPosition;
    position = startingPosition;
    velocity = 0;

    frequency = Math.max(0, frequency);

    k1 = damping / (Math.PI * frequency);
    k2 = 1 / Math.pow(2 * Math.PI * frequency, 2);
    k3 = (response * damping) / (2 * Math.PI * frequency);
  }

  public void SetValues(float Position, float Velocity) {
    position = Position;
    previousnput = Position;
    velocity = Velocity;
  }

  // If precise velocity is needed, and is available, copy func and replace TargetVelocity with a parameter
  public Vector<N2> Evaluate(double Target, double deltaTime) {
    double T = deltaTime;

    // Estimate Target Velocity
    double TargetVelocity = (Target - previousnput) / T;
    previousnput = Target;

    // Magic as far as I'm concerned
    double k2_stable = Math.max(Math.max(k2, T * T / 2 + T * k1 / 2), T * k1); // Clamp k2 if timestep is too low to be stable

    position = position + T * velocity; // Integrate position
    double acceleration = (Target + k3 * TargetVelocity - position - k1 * velocity) / k2_stable;
    velocity = velocity + T * acceleration; // Integrate velocity

    return VecBuilder.fill(position, velocity);
  }

  public Vector<N2> Evaluate(double Target, double TargetVelocity, double deltaTime) {
    double T = deltaTime;

    // Estimate Target Velocity
    // double TargetVelocity = (Target - previousnput) / T;
    previousnput = Target;

    // Magic as far as I'm concerned
    double k2_stable = Math.max(Math.max(k2, T * T / 2 + T * k1 / 2), T * k1); // Clamp k2 if timestep is too low to be stable

    position = position + T * velocity; // Integrate position
    double acceleration = (Target + k3 * TargetVelocity - position - k1 * velocity) / k2_stable;
    velocity = velocity + T * acceleration; // Integrate velocity

    return VecBuilder.fill(position, velocity);
  }
}