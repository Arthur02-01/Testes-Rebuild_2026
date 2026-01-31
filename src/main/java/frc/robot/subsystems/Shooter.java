package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Hardwares.HardwaresShooter;
import frc.robot.StatesMachines.StateMachineShooter;

public class Shooter extends SubsystemBase {

    private final HardwaresShooter io = new HardwaresShooter();
    private final StateMachineShooter sm = new StateMachineShooter();

    private ConstantesShooter.Velocidade velocidade =
        ConstantesShooter.Velocidade.NORMAL;

    /* ================= API ================= */

    public void atirarFrente(ConstantesShooter.Velocidade vel) {
        velocidade = vel;
        sm.set(StateMachineShooter.Estado.ATIRANDO_FRENTE);
    }

    public void atirarTras(ConstantesShooter.Velocidade vel) {
        velocidade = vel;
        sm.set(StateMachineShooter.Estado.ATIRANDO_TRAS);
    }

    public void parar() {
        sm.set(StateMachineShooter.Estado.PARADO);
    }

    public boolean estaAtivo() {
        return !sm.is(StateMachineShooter.Estado.PARADO);
    }

    /* ================= LOOP ================= */

    @Override
    public void periodic() {

        double v = velocidade.valor;

        switch (sm.get()) {

            case ATIRANDO_FRENTE -> {
                io.arlindo.set(-v);
                io.boquinha.set(+v);
            }

            case ATIRANDO_TRAS -> {
                io.arlindo.set(+v);
                io.boquinha.set(-v);
            }

            case PARADO -> {
                io.arlindo.stopMotor();
                io.boquinha.stopMotor();
            }
        }

        SmartDashboard.putString(
            "Shooter/Estado",
            sm.get().name()
        );

        SmartDashboard.putString(
            "Shooter/Velocidade",
            velocidade.name()
        );

        SmartDashboard.putNumber(
            "Shooter/Arlindo RPM",
            io.arlindoEncoder.getVelocity()
        );

        SmartDashboard.putNumber(
            "Shooter/Boquinha RPM",
            io.boquinhaEncoder.getVelocity()
        );
    }
}
