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

    public final SparkClosedLoopController arlindopid;
    public final SparkClosedLoopController boquinhapid;

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

        arlindopid = arlindo.getClosedLoopController();
        boquinhapid = boquinha.getClosedLoopController();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
           .smartCurrentLimit(60);

        cfg.closedLoop
        .p(0.00025)
        .i(0.0)
        .d(0.0)
        .velocityFF(1.0/5700.0)
        .outputRange(-1.0, 1.0);

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
