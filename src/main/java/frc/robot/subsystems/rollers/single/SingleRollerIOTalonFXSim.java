package frc.robot.subsystems.rollers.single;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SingleRollerIOTalonFXSim extends SingleRollerIOTalonFX {
  private final DCMotorSim motorSim;

  /** Motion Magic does _not_ work in Sim, so we use Position Voltage in the meantime */
  private final PositionVoltage positionVoltage;

  private final VelocityVoltage velocityVoltage;

  public SingleRollerIOTalonFXSim(
      int canId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig,
      DCMotor dcMotor,
      double moi) {
    super(canId, reduction, currentLimitAmps, invert, isBrakeMode, foc, gains, mmConfig);

    motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.00002, reduction), dcMotor);
    velocityVoltage = new VelocityVoltage(0).withSlot(0).withEnableFOC(foc);
    positionVoltage = new PositionVoltage(0).withSlot(0).withEnableFOC(foc);
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
