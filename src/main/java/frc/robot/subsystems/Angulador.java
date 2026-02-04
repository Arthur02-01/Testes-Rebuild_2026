package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constantes.ConstantesAngulador;
import frc.robot.Extras.AnguloPreset;
import frc.robot.Hardwares.HardwaresAngulador;
import frc.robot.Kinematics.KInematicsAngulador;
import frc.robot.StatesMachines.StateMachineAngulador;

    
public class Angulador extends SubsystemBase {

    private final HardwaresAngulador io = new HardwaresAngulador();
    private final StateMachineAngulador sm = new StateMachineAngulador();

    private final ArmFeedforward ff =
        new ArmFeedforward(
            Constants.FFAlinhador.kS,
            Constants.FFAlinhador.kG,
            Constants.FFAlinhador.kV
        );

    private final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(
            ConstantesAngulador.MAX_VEL,
            ConstantesAngulador.MAX_ACC
        );

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    @SuppressWarnings("unused")
    private double alvoGraus = Double.NaN;
    private double manualOutput = 0.0;
    private double testeOutput = 0.0;
    private double anguloHold = 0.0;
    public static final double ERRO_MAX_HOLD =  2.0;
    private double tempoSemMovimento = 0.0;
    private double ultimoTimestamp = Timer.getFPGATimestamp();

    /* ================= API ================= */

    public double getAngulo() {
        return KInematicsAngulador.rotacoesParaGraus(
            io.encoder.getPosition()
        );
    }
    public StateMachineAngulador.Estado getEstado() {
    return sm.get();
}

    public void moverParaAngulo(double graus) {
    double grausLimitado = MathUtil.clamp(
        graus,
        ConstantesAngulador.LIMITE_INFERIOR,
        ConstantesAngulador.LIMITE_SUPERIOR
        );
        alvoGraus = grausLimitado;
        goal = new TrapezoidProfile.State(grausLimitado, 0.0);

    setpoint = new TrapezoidProfile.State(
        getAngulo(),
        0.0
    );
    sm.set(StateMachineAngulador.Estado.PERFIL);
}


    public void moverParaPreset(AnguloPreset preset) {
    moverParaAngulo(preset.graus);
    }

    public void controleManual(double output) {
        manualOutput = output;
        sm.set(StateMachineAngulador.Estado.MANUAL);
    }

    public void testeMotor (double output){
        testeOutput = output;
        sm.set(StateMachineAngulador.Estado.TESTE);
    }

    public void resetarFalha(){
        tempoSemMovimento = 0.0;
        sm.set(StateMachineAngulador.Estado.HOLD);
        io.motor.stopMotor();
    }

    

    /* ================= LOOP ================= */

    @Override
    public void periodic() {

        switch (sm.get()) {

            case PERFIL -> executarPerfil();
            case HOLD   -> executarHold();
            case MANUAL -> io.motor.set(manualOutput);
            case TESTE -> io.motor.set(testeOutput);
            default     -> io.motor.stopMotor();
        }

        
        SmartDashboard.putNumber("Angulador/Angulo", getAngulo());
        SmartDashboard.putString("Angulador/Estado", sm.get().name());

        double agora = Timer.getFPGATimestamp();
    double dt = agora - ultimoTimestamp;
    if(dt <= 0.0) {
         dt = ConstantesAngulador.DT;
    }
    double erroSetpoint = Math.abs(setpoint.position - getAngulo());
    
    double velocidadeGraus = KInematicsAngulador.rotacoesParaGraus(io.encoder.getVelocity());

    if (sm.is(StateMachineAngulador.Estado.PERFIL)
            && erroSetpoint > ConstantesAngulador.ERRO_TOLERANCIA) {
            boolean perfilLento = Math.abs(setpoint.velocity) < ConstantesAngulador.VELOCIDADE_MIN;
            if (Math.abs(velocidadeGraus) < ConstantesAngulador.VELOCIDADE_MIN 
            && perfilLento){ tempoSemMovimento += dt;
            } else {
                tempoSemMovimento = 0.0;
            }
        } else {
            tempoSemMovimento = 0.0;
        }

        if (tempoSemMovimento >= ConstantesAngulador.TEMPO_MAX_TRAVADO) {
            sm.set(StateMachineAngulador.Estado.FALHA);
            io.motor.stopMotor();
        }

        ultimoTimestamp = agora;
    }

   private void executarPerfil() {
    TrapezoidProfile profile = new TrapezoidProfile(constraints);

    setpoint = profile.calculate(
        ConstantesAngulador.DT,
        setpoint,
        goal
    );

    double ffVolts = ff.calculate(
        Math.toRadians(setpoint.position),
        Math.toRadians(setpoint.velocity)
    );

    io.pid.setSetpoint(
        KInematicsAngulador.grausParaRotacoes(setpoint.position),
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ffVolts
    );

    if (Math.abs(goal.position - setpoint.position)
        < ConstantesAngulador.MARGEM_ERRO_BASE) {

        anguloHold = goal.position;
        sm.set(StateMachineAngulador.Estado.HOLD);
    }
}


    private void executarHold() {

    double anguloAtual = getAngulo();
    double erro = anguloHold - anguloAtual;

    double ffVolts = ff.calculate(
        Math.toRadians(anguloAtual),
        0.0
    );

    io.pid.setSetpoint(
        KInematicsAngulador.grausParaRotacoes(anguloHold),
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ffVolts
    );
    if (Math.abs(erro) > ERRO_MAX_HOLD) {
        setpoint = new TrapezoidProfile.State(
            anguloAtual,
            0.0
        );
        goal = new TrapezoidProfile.State(
            anguloHold,
            0.0
        );
        sm.set(StateMachineAngulador.Estado.PERFIL);
    }
}

}
