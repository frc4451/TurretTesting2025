package frc.robot.subsystems.rollers.single;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class SingleRollerTypes {
  public static record SingleRollerFXConfig(
    /** CAN ID */
      int canId,
      DCMotor gearbox,
      /** Reduction for Mechanism */
      double reduction,
      /** Moment of Inertia */
      double moi,
      /** Whether the motor is inverted */
      boolean invert,
      double currentLimitAmps,
      boolean isBrakeMode,
      Rotation2d intialPosition,
      Rotation2d minimumPosition,
      Rotation2d maximumPosition,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig,
      boolean foc) {}
}
