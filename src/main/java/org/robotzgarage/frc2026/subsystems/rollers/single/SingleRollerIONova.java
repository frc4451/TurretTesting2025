package org.robotzgarage.frc2026.subsystems.rollers.single;

import com.thethriftybot.Conversion;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.PIDSlot;
import org.robotzgarage.frc2026.util.MotorConfigs.ThriftyNovaConfig;

public class SingleRollerIONova implements SingleRollerIO {
  private final ThriftyNova nova;
  private final Conversion positionConversion;
  private final Conversion velocityConversion;
  private final double reduction;

  private double positionGoalRotations = 0.0;

  public SingleRollerIONova(ThriftyNovaConfig config) {
    nova = new ThriftyNova(config.canId());
    nova.factoryReset();

    this.reduction = config.reduction();

    // Handles conversion from position math to encoder units
    positionConversion = new Conversion(config.positionUnit(), config.encoderType());
    velocityConversion = new Conversion(config.velocityUnit(), config.encoderType());

    // Handle overall config
    nova.setMaxCurrent(CurrentType.SUPPLY, config.currentLimitAmpsSupply())
        .setMaxCurrent(CurrentType.STATOR, config.currentLimitAmpsStator())
        .setRampUp(config.rampRateUp())
        .setRampDown(config.rampRateDown())
        .setMaxOutput(config.maxOutput())
        .setBrakeMode(config.isBrakeMode())
        .setInversion(config.invert())
        .enableSoftLimits(config.enableSoftLimits())
        .usePIDSlot(PIDSlot.SLOT0);

    // Handle PID control
    nova.pid0.setP(config.pidConfig().p());
    nova.pid0.setI(config.pidConfig().i());
    nova.pid0.setD(config.pidConfig().d());
    nova.pid0.setFF(config.pidConfig().ff());

    // NOTE Find better place to put errors
    for (com.thethriftybot.ThriftyNova.Error error : nova.getErrors()) {
      System.err.println("Error" + error.toString());
    }

    nova.clearErrors();
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    // We need a more clear way to check this
    inputs.connected = nova.getID() != 0;

    inputs.positionRotations = positionConversion.fromMotor(nova.getPosition()) / reduction;
    inputs.velocityRotationsPerSec = velocityConversion.fromMotor(nova.getVelocity()) / reduction;

    inputs.appliedVoltage = nova.getVoltage();
    inputs.supplyCurrentAmps = nova.getSupplyCurrent();
    inputs.torqueCurrentAmps = nova.getStatorCurrent();
    inputs.temperatureCelsius = nova.getTemperature();

    inputs.positionGoalRotations = positionGoalRotations / reduction;
    inputs.positionSetpointRotations = nova.getSetPoint() / reduction;
  }

  @Override
  public void runVolts(double volts) {
    this.nova.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityMetersPerSec) {
    this.nova.setVelocity(velocityMetersPerSec);
  }

  @Override
  public void setGoal(double positionRotations) {
    positionGoalRotations = positionRotations * reduction;
    this.nova.setPosition(positionConversion.toMotor(positionRotations));
  }

  @Override
  public void resetPosition(double positionRotations) {
    this.nova.setPosition(positionRotations);
  }

  @Override
  public void stop() {
    this.nova.stopMotor();
  }
}
