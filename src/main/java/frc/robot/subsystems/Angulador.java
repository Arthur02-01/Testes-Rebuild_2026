package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constantes.ConstantesAngulador;
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

    private double alvoGraus = Double.NaN;
    private double manualOutput = 0.0;
    private double anguloHold = 0.0;

    /* ================= API ================= */

    public double getAngulo() {
        return KInematicsAngulador.rotacoesParaGraus(
            io.encoder.getPosition()
        );
    }

    public void moverParaAngulo(double graus) {
        alvoGraus = graus;
        goal = new TrapezoidProfile.State(graus, 0.0);
        sm.set(StateMachineAngulador.Estado.PERFIL);
    }

    public void controleManual(double output) {
        manualOutput = output;
        sm.set(StateMachineAngulador.Estado.MANUAL);
    }

    

    /* ================= LOOP ================= */

    @Override
    public void periodic() {

        switch (sm.get()) {

            case PERFIL -> executarPerfil();
            case HOLD   -> executarHold();
            case MANUAL -> io.motor.set(manualOutput);
            default     -> io.motor.stopMotor();
        }

        
        SmartDashboard.putNumber("Angulador/Angulo", getAngulo());
        SmartDashboard.putString("Angulador/Estado", sm.get().name());
    }

    private void executarPerfil() {
        TrapezoidProfile profile = new TrapezoidProfile(constraints);
        setpoint = profile.calculate(
            ConstantesAngulador.DT, setpoint, goal
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
        io.pid.setSetpoint(
            KInematicsAngulador.grausParaRotacoes(anguloHold),
            SparkBase.ControlType.kPosition
        );
    }
}
