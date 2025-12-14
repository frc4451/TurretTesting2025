package org.robotzgarage.frc2026.subsystems.rollers.single;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXSConfig;

public class SingleRollerIOSimTalonFXS extends SingleRollerIOTalonFXS {
  private final DCMotorSim motorSim;

  /** Motion Magic does _not_ work in Sim, so we use Position Voltage in the meantime */
  private final PositionVoltage positionVoltage;

  /** Motion Magic does _not_ work in Sim, so we use Velocity Voltage in the meantime */
  private final VelocityVoltage velocityVoltage;

  public SingleRollerIOSimTalonFXS(TalonFXSConfig config) {
    super(config);

    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(config.dcMotor(), config.moi(), reduction),
            config.dcMotor());
    velocityVoltage = new VelocityVoltage(0).withSlot(0).withEnableFOC(config.foc());
    positionVoltage = new PositionVoltage(0).withSlot(0).withEnableFOC(config.foc());
  }

  @Override
  public void setGoal(double positionRotations) {
    positionGoalRotations = positionRotations * reduction;
    talon.setControl(positionVoltage.withPosition(positionGoalRotations));
  }

  @Override
  public void setVelocity(double velocityRotationsPerSec) {
    talon.setControl(velocityVoltage.withVelocity(velocityRotationsPerSec * reduction));
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    // Still do everything as expected for TalonFXS
    super.updateInputs(inputs);

    // Update motor output from sim state
    motorSim.setInput(talon.getSimState().getMotorVoltage());
    motorSim.update(Constants.loopPeriodSecs);

    // (WIP) Take DCMotorSim configs and set them to TalonFXS Sim
    talon.getSimState().setRawRotorPosition(motorSim.getAngularPositionRotations() / reduction);
    talon
        .getSimState()
        .setRotorVelocity(
            Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec() / reduction));
  }
}
