package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRoller;
import org.robotzgarage.frc2026.subsystems.rollers.single.SingleRollerIO;

/** Simulation support for a single motor that only spins. */
public class Flywheel extends SingleRoller {
  /**
   * Radius of flywheel, necessary for velocity calculations In the event that you use differently
   * sized flywheels, they should be separate Flywheel objects with their own respected
   * measurements.
   */
  private final double radiusMeters;

  private double lastPositionRotations = 0.0;

  private final Optional<FlywheelSim> flywheelSim;

  public Flywheel(String name, SingleRollerIO io, double radiusMeters) {
    super(name, io);
    this.radiusMeters = radiusMeters;

    if (RobotBase.isSimulation()) {
      flywheelSim =
          Optional.of(
              new FlywheelSim(
                  LinearSystemId.createFlywheelSystem(
                      FlywheelConstants.gearbox,
                      FlywheelConstants.moi,
                      FlywheelConstants.reduction),
                  FlywheelConstants.gearbox));
    } else {
      flywheelSim = Optional.empty();
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        name + "/PositionDeltaRotations", inputs.positionRotations - lastPositionRotations);

    lastPositionRotations = inputs.positionRotations;

    // Flywheel Simulation, if present
    flywheelSim.ifPresent(
        flywheelSim -> {
          flywheelSim.setInputVoltage(inputs.appliedVoltage);
          flywheelSim.setAngularVelocity(Units.rotationsToRadians(inputs.velocityRotationsPerSec));

          String logRoot = name + "/PhysicalSim";
          Logger.recordOutput(
              logRoot + "/VelocityRotPerSec",
              flywheelSim.getAngularVelocity().in(RotationsPerSecond));
          Logger.recordOutput(
              logRoot + "/OmegaRotPerSecSq",
              flywheelSim.getAngularAcceleration().in(RotationsPerSecondPerSecond));
          Logger.recordOutput(logRoot + "/TorqueNwtMeters", flywheelSim.getTorqueNewtonMeters());
          Logger.recordOutput(logRoot + "/CurrentDrawAmps", flywheelSim.getCurrentDrawAmps());
          Logger.recordOutput(logRoot + "/InputVoltage", flywheelSim.getInputVoltage());
        });
  }

  public void setVelocity(double velocityMetersPerSec) {
    // CTRE has documentation to support this
    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters

    // We devide distance measured by circumference to "unrawp"
    double unwrappedRadiusMeters = (2 * Math.PI * radiusMeters);

    // Handle w = v / 2πr
    double velocityRotationsPerSec = velocityMetersPerSec / unwrappedRadiusMeters;

    io.setVelocity(velocityRotationsPerSec);
  }

  public void setVoltage(double volts) {
    io.runVolts(volts);
  }
}
