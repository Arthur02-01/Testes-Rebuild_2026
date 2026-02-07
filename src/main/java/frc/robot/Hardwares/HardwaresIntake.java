package frc.robot.Hardwares;

import frc.robot.Constantes.ConstantesIntakeFloor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

@SuppressWarnings ("removal")
public class HardwaresIntake {

    
    public final SparkMax IntakeMotor;
    public final SparkMax PivotMotor;

    public final SparkClosedLoopController pid;
    public final RelativeEncoder encoder_pivot;

    public HardwaresIntake() {
        IntakeMotor = new SparkMax(
            ConstantesIntakeFloor.IntakeFloorMotor.IntakeMotor,
            MotorType.kBrushless
        );
        PivotMotor = new SparkMax(
            ConstantesIntakeFloor.PivotMotor.PivotMotor,
            MotorType.kBrushless
        );
        encoder_pivot = PivotMotor.getEncoder();
        pid = PivotMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);

        config.smartCurrentLimit(60);

        PivotMotor.setInverted(true);

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
           .smartCurrentLimit(60);

        cfg.closedLoop
        .p(0.8)
        .i(0.0)
        .d(0.02);

        IntakeMotor.configure(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        ); 
        PivotMotor.configure(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    } 
}