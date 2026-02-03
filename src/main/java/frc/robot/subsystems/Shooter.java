package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Hardwares.HardwaresShooter;
import frc.robot.StatesMachines.StateMachineShooter;

public class Shooter extends SubsystemBase {

    private final HardwaresShooter io = new HardwaresShooter();
    private final StateMachineShooter sm = new StateMachineShooter();

    private double rpmAlvo = 0.0;

    private ConstantesShooter.Velocidade velocidade =
        ConstantesShooter.Velocidade.NORMAL;

    /* ================= API ================= */

    /** Apenas muda a velocidade (NÃO gira) */
    public void setVelocidade(ConstantesShooter.Velocidade vel) {
        this.velocidade = vel;
        this.rpmAlvo = vel.rpm;
    }

    /** Ativa giro para frente */
    public void atirarFrente() {
        if (sm.is(StateMachineShooter.Estado.ATIRANDO_FRENTE)) {
            sm.set(StateMachineShooter.Estado.PARADO);
        } else {
            sm.set(StateMachineShooter.Estado.ATIRANDO_FRENTE);
        }
    }

    /** Ativa giro para trás */
    public void atirarTras() {
       if (sm.is(StateMachineShooter.Estado.ATIRANDO_TRAS)){
        sm.set(StateMachineShooter.Estado.PARADO);
       } else {
        sm.set(StateMachineShooter.Estado.ATIRANDO_TRAS);
       }
    }

    public ConstantesShooter.Velocidade getVelocidade() {
    return velocidade;
    }

    /** Para completamente */
    public void parar() {
        sm.set(StateMachineShooter.Estado.PARADO);
    }

    public boolean estaAtivo() {
        return !sm.is(StateMachineShooter.Estado.PARADO);
    }
    public boolean pronto(){
        return 
            Math.abs(rpmAlvo - io.arlindoEncoder.getVelocity()) < ConstantesShooter.TOLERANCIA_RPM &&
            Math.abs(rpmAlvo - io.boquinhaEncoder.getVelocity()) < ConstantesShooter.TOLERANCIA_RPM;
    }

    /* ================= LOOP ================= */

    @Override
    public void periodic() {

        switch (sm.get()) {

            case ATIRANDO_FRENTE -> {
                io.arlindopid.setSetpoint(
                    -rpmAlvo, ControlType.kVelocity
                );
                io.boquinhapid.setSetpoint(
                    +rpmAlvo, ControlType.kVelocity
                );
            }

            case ATIRANDO_TRAS -> {
                io.arlindopid.setSetpoint(
                    +rpmAlvo, ControlType.kVelocity
                );
                io.boquinhapid.setSetpoint(
                    -rpmAlvo, ControlType.kVelocity
                );
            }

            case PARADO -> {
                io.arlindo.stopMotor();
                io.boquinha.stopMotor();
            }
        }

         SmartDashboard.putString("Shooter/Estado", sm.get().name());
        SmartDashboard.putNumber("Shooter/RPM Alvo", rpmAlvo);
        SmartDashboard.putNumber("Shooter/Arlindo RPM", io.arlindoEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/Boquinha RPM", io.boquinhaEncoder.getVelocity());
    }
}
