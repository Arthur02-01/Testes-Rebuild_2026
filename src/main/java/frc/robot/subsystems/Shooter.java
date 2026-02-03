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

    private double rpmAlvo = ConstantesShooter.Velocidade.NORMAL.rpm;
    private double ultimoSetpointArlindo = Double.NaN;
    private double ultimoSetpointBoquinha = Double.NaN;
    private boolean pronto = false;

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
        boolean dentro =
            Math.abs(rpmAlvo - io.arlindoEncoder.getVelocity())
                < ConstantesShooter.TOLERANCIA_RPM
            && Math.abs(rpmAlvo - io.boquinhaEncoder.getVelocity())
                < ConstantesShooter.TOLERANCIA_RPM;

        boolean fora =
            Math.abs(rpmAlvo - io.arlindoEncoder.getVelocity())
                > ConstantesShooter.TOLERANCIA_RPM_SAIDA
            || Math.abs(rpmAlvo - io.boquinhaEncoder.getVelocity())
                > ConstantesShooter.TOLERANCIA_RPM_SAIDA;

        if (!pronto && dentro) {
            pronto = true;
        } else if (pronto && fora) {
            pronto = false;
        }

        return pronto;
    }

    private void definirSetpoint(double arlindoRpm, double boquinhaRpm) {
        if (Double.isNaN(ultimoSetpointArlindo)
            || ultimoSetpointArlindo != arlindoRpm) {
            io.arlindopid.setSetpoint(arlindoRpm, ControlType.kVelocity);
            ultimoSetpointArlindo = arlindoRpm;
        }
        if (Double.isNaN(ultimoSetpointBoquinha)
            || ultimoSetpointBoquinha != boquinhaRpm) {
            io.boquinhapid.setSetpoint(boquinhaRpm, ControlType.kVelocity);
            ultimoSetpointBoquinha = boquinhaRpm;
        }
    }
    
        
    

    /* ================= LOOP ================= */

    @Override
    public void periodic() {

        switch (sm.get()) {

            case ATIRANDO_FRENTE -> {
                definirSetpoint(-rpmAlvo, +rpmAlvo);
            }

            case ATIRANDO_TRAS -> {
                definirSetpoint(+rpmAlvo, -rpmAlvo);
            }

            case PARADO -> {
                io.arlindo.stopMotor();
                io.boquinha.stopMotor();
                ultimoSetpointArlindo = Double.NaN;
                ultimoSetpointBoquinha = Double.NaN;
                pronto = false;
            }
        }

         SmartDashboard.putString("Shooter/Estado", sm.get().name());
        SmartDashboard.putNumber("Shooter/RPM Alvo", rpmAlvo);
        SmartDashboard.putNumber("Shooter/Arlindo RPM", io.arlindoEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/Boquinha RPM", io.boquinhaEncoder.getVelocity());
    }
}
