package frc.robot.subsystems.rollers.single;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class SingleRollerIOTalonFXS implements SingleRollerIO {
  protected final TalonFXS talon;
  protected final double reduction;

  protected final StatusSignal<Angle> position;
  protected final StatusSignal<AngularVelocity> velocity;
  protected final StatusSignal<Voltage> voltage;
  protected final StatusSignal<Current> supplyCurrentAmps;
  protected final StatusSignal<Current> torqueCurrentAmps;
  protected final StatusSignal<Temperature> tempCelsius;

  protected final StatusSignal<Double> positionSetpointRotations;
  protected final StatusSignal<Double> velocitySetpointRotationsPerSec;

  protected final MotionMagicVoltage mmVoltage;
  protected final MotionMagicVelocityVoltage mmVelocityVoltage;

  protected final VoltageOut voltageOut;
  protected final DutyCycleOut dutyCycle;
  protected final NeutralOut neutralOut = new NeutralOut();

  protected double positionGoalRotations = 0;

  public SingleRollerIOTalonFXS(
      int canId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig) {
    this.reduction = reduction;

    talon = new TalonFXS(canId, Constants.alternateCanBus);

    voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0).withEnableFOC(foc);
    dutyCycle = new DutyCycleOut(0).withUpdateFreqHz(0).withEnableFOC(foc);
    mmVoltage = new MotionMagicVoltage(0.0).withUpdateFreqHz(0).withEnableFOC(foc);
    mmVelocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(foc);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    voltage = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    positionSetpointRotations = talon.getClosedLoopReference();
    velocitySetpointRotationsPerSec = talon.getClosedLoopReferenceSlope();

    TalonFXSConfiguration cfg = new TalonFXSConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(currentLimitAmps);
    cfg.Slot0 = gains;
    cfg.MotionMagic = mmConfig;
    // spotless:on

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.phoenixUpdateFreqHz,
        position,
        velocity,
        voltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        tempCelsius,
        positionSetpointRotations,
        velocitySetpointRotationsPerSec);
    talon.optimizeBusUtilization(0.0, 1.0);

    talon.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                voltage,
                supplyCurrentAmps,
                torqueCurrentAmps,
                tempCelsius,
                positionSetpointRotations,
                velocitySetpointRotationsPerSec)
            .isOK();

    inputs.positionRotations = position.getValueAsDouble() / reduction;
    inputs.velocityRotationsPerSec = velocity.getValueAsDouble() / reduction;

    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = tempCelsius.getValueAsDouble();

    inputs.positionGoalRotations = positionGoalRotations / reduction;
    inputs.positionSetpointRotations = positionSetpointRotations.getValueAsDouble() / reduction;
    inputs.velocitySetpointRotationsPerSec =
        velocitySetpointRotationsPerSec.getValueAsDouble() / reduction;
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  //   @Override
  //   public void setVelocity(double velocity) {
  //     talon.setControl(dutyCycle.withOutput(velocity * reduction));
  //   }
  @Override
  public void setVelocity(double velocityRotationsPerSec) {
    // Convert degrees/sec to rotations/sec
    // double velocityRadPerSec = Units.degreesToRadians(velocity);
    // double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    // double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    // motor.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
    talon.setControl(mmVelocityVoltage.withVelocity(velocityRotationsPerSec / reduction));
  }

  @Override
  public void setGoal(double positionRotations) {
    positionGoalRotations = positionRotations * reduction;
    talon.setControl(mmVoltage.withPosition(positionGoalRotations));
  }

  @Override
  public void resetPosition(double positionRad) {
    talon.setPosition(positionRad * reduction);
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }
}
