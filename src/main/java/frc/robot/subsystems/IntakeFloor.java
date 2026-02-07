package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;

import frc.robot.Hardwares.HardwaresIntake;
import frc.robot.Constantes.ConstantesAngulador;
import frc.robot.Constantes.ConstantesIntakeFloor;
import frc.robot.Extras.AngulosPresetPivot;
import frc.robot.StatesMachines.StateMachineIntakeFloor;
import frc.robot.Kinematics.KInematicsAngulador;
import frc.robot.Kinematics.KinematicsIntakeFloor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

@SuppressWarnings ("unused")
public class IntakeFloor extends SubsystemBase {

    private static final double VELOCIDADE_MAX_INTAKE = 0.90;

    private boolean IntakeGirando = false;


    private final HardwaresIntake io = new HardwaresIntake();
    private final StateMachineIntakeFloor sm = new StateMachineIntakeFloor();

    private final ArmFeedforward ff =
        new ArmFeedforward(ConstantesIntakeFloor.FFPivot.kS,
        ConstantesIntakeFloor.FFPivot.kG,
        ConstantesIntakeFloor.FFPivot.kV
        );

    private final TrapezoidProfile.Constraints constraints = 
        new TrapezoidProfile.Constraints(
            ConstantesIntakeFloor.MAX_VEL_PIVOT,
            ConstantesIntakeFloor.MAX_ACC_PIVOT
        );

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    
    private double alvoGraus = Double.NaN;
    private double manualOutput = 0.0;
    private double testeOutput = 0.0;
    private double anguloHold = 0.0;
    public static final double ERRO_MAX_HOLD =  2.0;
    private double tempoSemMovimento = 0.0;
    private double ultimoTimestamp = Timer.getFPGATimestamp();

    public double getAnguloPivot(){
        return KinematicsIntakeFloor.rotacoesParaGrausPivot(
            io.encoder_pivot.getPosition()
        );
    }

    public StateMachineIntakeFloor.EstadoPivot getEstadoPivot() {
        return sm.get();
    }

    public void moverParaAnguloPivot(double graus) {
        double grausLimitado = MathUtil.clamp(
            graus,
            ConstantesIntakeFloor.LIMITE_INFERIOR_PIVOT,
            ConstantesIntakeFloor.LIMITE_SUPERIOR_PIVOT
        );
        alvoGraus = grausLimitado;
        goal = new TrapezoidProfile.State(grausLimitado, 0.0);

    setpoint = new TrapezoidProfile.State(
        getAnguloPivot(),
        0.0
    );
    sm.set(StateMachineIntakeFloor.EstadoPivot.PERFIL);
    }

    public void moverParaPreset (AngulosPresetPivot preset) {
        moverParaAnguloPivot(preset.graus);
    }

    public void controleManual(double output) {
        manualOutput = output;
        sm.set(StateMachineIntakeFloor.EstadoPivot.MANUAL);
    }

    public void testeMotor (double output) {
        testeOutput = output;
        sm.set(StateMachineIntakeFloor.EstadoPivot.TESTE);
    }

    public void resetarFalha(){
        tempoSemMovimento = 0.0;
        double anguloAtual = getAnguloPivot();
        anguloHold = anguloAtual;
        goal = new TrapezoidProfile.State(anguloAtual, 0.0);
        setpoint = new TrapezoidProfile.State(anguloAtual, 0.0);
        sm.set(StateMachineIntakeFloor.EstadoPivot.HOLD);
    }

    public void IntakeVelocidade(double velocidade) {

        double velocidadeLimitada =
            MathUtil.clamp(velocidade, -VELOCIDADE_MAX_INTAKE, VELOCIDADE_MAX_INTAKE);

        io.IntakeMotor.set(velocidadeLimitada);

        IntakeGirando = Math.abs(velocidadeLimitada) > 0.02;
    }

    public void IntakeOn(){
        IntakeVelocidade(VELOCIDADE_MAX_INTAKE);
    }

    public void IntakeReverse(){
        IntakeVelocidade(-VELOCIDADE_MAX_INTAKE);
    }

    public void PararIntake(){
        ParaIntake();
    }

    public void ParaIntake() {

        io.IntakeMotor.set(0);

        IntakeGirando = false;
    }

    @Override
    public void periodic() {

        switch (sm.get()) {
            case PERFIL -> executarPerfil();
            case HOLD -> executarHold();
            case MANUAL -> io.PivotMotor.set(manualOutput);
            case TESTE -> io.PivotMotor.set(testeOutput);
            default     -> io.PivotMotor.stopMotor();
        }

        SmartDashboard.putBoolean("Intake girando", IntakeGirando);
        SmartDashboard.putNumber("Pivot/Angulo", getAnguloPivot());
        SmartDashboard.putString("Pivot/Estado", sm.get().name());

        double agora = Timer.getFPGATimestamp();
    double dt = agora - ultimoTimestamp;
    if (dt <= 0.0) {
         dt = ConstantesAngulador.DT;
    }
    double erroSetpoint = Math.abs(setpoint.position - getAnguloPivot());
    
    double velocidadeGraus = KinematicsIntakeFloor.rotacoesParaGrausPivot(io.encoder_pivot.getVelocity());

    if (sm.is(StateMachineIntakeFloor.EstadoPivot.PERFIL)
    && erroSetpoint > ConstantesIntakeFloor.ERRO_TOLERANCIA_PIVOT){
    boolean perfilLento = Math.abs(setpoint.velocity) < ConstantesIntakeFloor.VELOCIDADE_MIN_PIVOT;
    if (Math.abs(velocidadeGraus) < ConstantesIntakeFloor.VELOCIDADE_MIN_PIVOT
    && perfilLento){ tempoSemMovimento += dt;
    } else {
        tempoSemMovimento = 0.0;
    }
} else {
    tempoSemMovimento = 0.0;
}
    if (tempoSemMovimento >= ConstantesIntakeFloor.TEMPO_MAX_TRAVADO_PIVOT) {
        sm.set(StateMachineIntakeFloor.EstadoPivot.FALHA);
        io.PivotMotor.stopMotor();
    }
        ultimoTimestamp = agora;
    }

private void executarPerfil() {
 TrapezoidProfile profile = new TrapezoidProfile(constraints);

 setpoint = profile.calculate(
    ConstantesIntakeFloor.DT_PIVOT,
    setpoint,
    goal
  );

  double ffVolts = ff.calculate(Math.toRadians(setpoint.position), Math.toRadians(setpoint.velocity)
  );

  io.pid.setSetpoint(KinematicsIntakeFloor.grausParaRotacoesPivot(setpoint.position),
   SparkBase.ControlType.kPosition,
   ClosedLoopSlot.kSlot0,
   ffVolts
   );

    if (Math.abs(goal.position - setpoint.position)
    < ConstantesIntakeFloor.MARGEM_ERRO_BASE_PIVOT
    ) {
        anguloHold = goal.position;
        sm.set(StateMachineIntakeFloor.EstadoPivot.HOLD);
    }
}
private void executarHold() {

    double anguloAtual = getAnguloPivot();
    double erro = anguloHold - anguloAtual;

    double ffVolts = ff.calculate(
        Math.toRadians(anguloAtual),
        0.0
    );

    io.pid.setSetpoint(
        KinematicsIntakeFloor.grausParaRotacoesPivot(anguloHold),
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
        sm.set(StateMachineIntakeFloor.EstadoPivot.PERFIL);
    }
}
}