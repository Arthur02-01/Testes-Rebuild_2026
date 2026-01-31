package frc.robot.Hardwares;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class HardwaresShooter {

    public final SparkMax arlindo;
    public final SparkMax boquinha;

    public final RelativeEncoder arlindoEncoder;
    public final RelativeEncoder boquinhaEncoder;

    public HardwaresShooter() {

        arlindo = new SparkMax(
            Constants.Shooter.ShooterArlindo,
            SparkLowLevel.MotorType.kBrushless
        );

        boquinha = new SparkMax(
            Constants.Shooter.ShooterBoquinha,
            SparkLowLevel.MotorType.kBrushless
        );

        arlindoEncoder = arlindo.getEncoder();
        boquinhaEncoder = boquinha.getEncoder();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
           .smartCurrentLimit(60);

        arlindo.configure(cfg,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters
        );

        boquinha.configure(cfg,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters
        );
    }
}
