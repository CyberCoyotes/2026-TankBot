package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class PhoenixUtil {

    private PhoenixUtil() {}

    /**
     * Apply standard drive motor config: inversion, coast mode, 40 A supply limit.
     */
    public static void configureDriveMotor(TalonFXConfigurator cfg, InvertedValue inversion) {
        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inversion;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.apply(config);
    }
}
