package org.robotzgarage.frc2026.util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.thethriftybot.Conversion.PositionUnit;
import com.thethriftybot.Conversion.VelocityUnit;
import com.thethriftybot.ThriftyNova.EncoderType;
import edu.wpi.first.math.system.plant.DCMotor;
import lombok.Builder;
import org.robotzgarage.frc2026.subsystems.rollers.feedforward_controller.FeedforwardController;

public class MotorConfigs {
  @Builder
  public record SimMotorConfig(
      DCMotor motorModel,
      double reduction,
      double moi,
      double maxVelocity,
      double maxAcceleration,
      double kP,
      double kI,
      double kD,
      FeedforwardController ff) {}

  @Builder
  public record TalonFXConfig(
      int canId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      Slot0Configs slot0Config,
      MotionMagicConfigs mmConfig,
      String canbus,
      DCMotor dcMotor,
      double moi) {
    public TalonFXConfig {
      // Assume that if no argument is provided, we're using the RIO bus.
      if (canbus == null) {
        canbus = "rio";
      }

      if (dcMotor == null) {
        dcMotor = DCMotor.getFalcon500Foc(1);
      }
    }
  }

  @Builder
  public record TalonFXSConfig(
      int canId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig,
      String canbus,
      DCMotor dcMotor,
      double moi) {
    public TalonFXSConfig {
      // Assume that if no argument is provided, we're using the RIO bus.
      if (canbus == null) {
        canbus = "rio";
      }

      if (dcMotor == null) {
        // This will never be a Kraken x60, but it's temporary until Minion is
        // added to the DCMotor library
        dcMotor = DCMotor.getKrakenX60Foc(1);
      }
    }
  }

  /** Shorthand PID config for any generic PID controller */
  @Builder
  public record GenericPIDConfig(double p, double i, double d, double ff) {}

  @Builder
  public record ThriftyNovaConfig(
      int canId,
      double reduction,
      double currentLimitAmpsSupply,
      double currentLimitAmpsStator,
      double rampRateUp,
      double rampRateDown,
      double maxOutput,
      boolean invert,
      boolean isBrakeMode,
      boolean enableSoftLimits,
      GenericPIDConfig pidConfig,
      PositionUnit positionUnit,
      VelocityUnit velocityUnit,
      EncoderType encoderType) {
    public ThriftyNovaConfig {
      // Default should be ROTATIONS to stay consistent with CTR-E logic
      if (positionUnit == null) {
        positionUnit = PositionUnit.ROTATIONS;
      }

      // Default should be ROTATIONS/SEC to stay consistent with CTR-E logic
      if (velocityUnit == null) {
        velocityUnit = VelocityUnit.ROTATIONS_PER_SEC;
      }

      // Assume we use the internal encoder if not specified otherwise
      if (encoderType == null) {
        encoderType = EncoderType.INTERNAL;
      }

      if (pidConfig == null) {
        pidConfig = GenericPIDConfig.builder().p(1).i(0).d(0).ff(0).build();
      }
    }
  }
}
