package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
public class IntakeFloor extends SubsystemBase {
    
    // Objeto que representa o motor do intake
    private final SparkMax IntakeMotor;

    // Velocidade máxima permitida para o intake (25%)
    private static final double VELOCIDADE_MAX = 0.25;

    // Flag para indicar se o intake está girando
    private boolean IntakeGirando = false;

    // Construtor do subsystem
    public IntakeFloor() {

        // Cria o Spark Max usando o ID definido em Constants
        IntakeMotor = new SparkMax(
            Constants.IntakeFloorMotor.IntakeMotor,
            MotorType.kBrushless
        );

        // Cria o objeto de configuração do Spark Max
        SparkMaxConfig config = new SparkMaxConfig();

        // Define o motor para modo Brake (para parar rapidamente)
        config.idleMode(IdleMode.kBrake);

        // Define limite de corrente em 60A para proteção
        config.smartCurrentLimit(60);

        IntakeMotor.configure(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );        
    }

    // Define a velocidade do intake
    public void IntakeVelocidade(double velocidade) {

        // Limita a velocidade entre -VELOCIDADE_MAX e +VELOCIDADE_MAX
        double velocidadeLimitada =
            MathUtil.clamp(velocidade, -VELOCIDADE_MAX, VELOCIDADE_MAX);

        // Aplica a velocidade no motor
        IntakeMotor.set(velocidadeLimitada);

        // Considera o intake girando se a velocidade for maior que um deadband
        IntakeGirando = Math.abs(velocidadeLimitada) > 0.02;
    }

    // Para completamente o intake
    public void ParaIntake() {

        // Zera a velocidade do motor
        IntakeMotor.set(0);

        // Atualiza o estado interno
        IntakeGirando = false;
    }

    // Método chamado automaticamente a cada ciclo do robô
    @Override
    public void periodic() {

        // Envia para o SmartDashboard se o intake está girando
        SmartDashboard.putBoolean("Intake girando", IntakeGirando);
    }
}