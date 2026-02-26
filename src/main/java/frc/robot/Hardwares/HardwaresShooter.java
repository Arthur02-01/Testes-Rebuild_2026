package frc.robot.Hardwares;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constantes.ConstantesShooter;

@SuppressWarnings("unused")
public class HardwaresShooter {

    public final SparkMax arlindo;

    public final RelativeEncoder arlindoEncoder;

    public final SparkClosedLoopController arlindopid;

    @SuppressWarnings("removal" )
    public HardwaresShooter() {

        arlindo = new SparkMax(
            ConstantesShooter.Shooter.ShooterArlindo,
            SparkLowLevel.MotorType.kBrushless
        );

        arlindoEncoder = arlindo.getEncoder();


        arlindopid = arlindo.getClosedLoopController();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kCoast)
           .smartCurrentLimit(60)
           .inverted(true);
        cfg.encoder
           .velocityConversionFactor(1.0 )
           .positionConversionFactor(1.0 );

        cfg.closedLoop
        .p(0.00045)
        .i(0.0)
        .d(0.00001)
        .iZone(ConstantesShooter.PID_IZONE_RPM)
        .velocityFF(ConstantesShooter.FF_VELOCIDADE)
        .outputRange(-1.0, 1.0);
        cfg.closedLoopRampRate(0.0);

        arlindo.configure(cfg,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters
        );
    }
}
