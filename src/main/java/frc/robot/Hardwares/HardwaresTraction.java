package frc.robot.Hardwares;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import frc.robot.Constantes.ConstantesTraction;

@SuppressWarnings ("unused")
public class HardwaresTraction {

    public RelativeEncoder leftEncoder;
    public RelativeEncoder rightEncoder;

    public SparkMax rightMotorFront =
            new SparkMax(ConstantesTraction.TractionConstants.rightFrontMotorID, MotorType.kBrushed);

    public SparkMax rightMotorBack =
            new SparkMax(ConstantesTraction.TractionConstants.rightBackMotorID, MotorType.kBrushed);

    public SparkMax leftMotorFront =
            new SparkMax(ConstantesTraction.TractionConstants.leftFrontMotorID, MotorType.kBrushed);

    public SparkMax leftMotorBack =
            new SparkMax(ConstantesTraction.TractionConstants.leftBackMotorID, MotorType.kBrushed);

    public SparkMaxConfig configSparkMotorEsquerda = new SparkMaxConfig();
    public SparkMaxConfig configSparkMotorDireita = new SparkMaxConfig();

        @SuppressWarnings("removal")
    public MotorControllerGroup leftMotorControllerGroup =
            new MotorControllerGroup(leftMotorFront, leftMotorBack);

    @SuppressWarnings("removal")
    public MotorControllerGroup rightMotorControllerGroup =
            new MotorControllerGroup(rightMotorFront, rightMotorBack);
        
    private final Pigeon2 pigeon = new Pigeon2(22);

    @SuppressWarnings("removal")
public HardwaresTraction() {

        pigeon.reset();

        // Configuração motores da direita
        configSparkMotorDireita
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);

        rightMotorFront.configure(
                configSparkMotorDireita,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        rightMotorBack.configure(
                configSparkMotorDireita,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Configuração motores da esquerda
        configSparkMotorEsquerda
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);

        leftMotorFront.configure(
                configSparkMotorEsquerda,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        leftMotorBack.configure(
                configSparkMotorEsquerda,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Inicializa encoders do Spark
        leftEncoder = leftMotorFront.getEncoder();
        rightEncoder = rightMotorFront.getEncoder();

        leftMotorControllerGroup.setInverted(false);
        rightMotorControllerGroup.setInverted(true);
    }
}
