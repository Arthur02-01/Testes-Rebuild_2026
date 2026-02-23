package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Hardwares.HardwaresShooter;
import frc.robot.StatesMachines.StateMachineShooter;

public class Shooter extends SubsystemBase {

    private final HardwaresShooter io = new HardwaresShooter();
    private final StateMachineShooter sm = new StateMachineShooter();

    private double rpmAlvo = ConstantesShooter.Velocidade.NORMAL.rpm;
    private double ultimoSetpoint = Double.NaN;

    private boolean alimentando = false;

    private final LinearFilter filtroDashboard =
        LinearFilter.singlePoleIIR(ConstantesShooter.FILTRO_RPM_TAU_S, 0.02);

    private double rpmFiltradoDashboard = 0.0;

        private ConstantesShooter.Velocidade velocidade =
        ConstantesShooter.Velocidade.NORMAL;

    /* ================= API ================= */

    public void setVelocidade(ConstantesShooter.Velocidade vel) {
        velocidade = vel;
        rpmAlvo = vel.rpm;
    }

    public void setAlimentando(boolean valor) {
        alimentando = valor;
    }

    public void setRpmDireto(double rpm) {
    rpmAlvo = rpm;
    }

    public void atirarFrente() {
    sm.set(StateMachineShooter.Estado.ATIRANDO_FRENTE);
    }
    public void atirarTras() {
    sm.set(StateMachineShooter.Estado.ATIRANDO_TRAS);
    }

    public void parar() {
        sm.set(StateMachineShooter.Estado.PARADO);
    }

    public boolean pronto() {
        double rpm = Math.abs(io.arlindoEncoder.getVelocity());
        return Math.abs(rpmAlvo - rpm) < ConstantesShooter.TOLERANCIA_RPM;
    }
    
    public ConstantesShooter.Velocidade getVelocidade() {
    return velocidade;
    }

    /* ================= LOOP ================= */

    @Override
    public void periodic() {

        double rpmBruto = Math.abs(io.arlindoEncoder.getVelocity());
        rpmFiltradoDashboard = filtroDashboard.calculate(rpmBruto);

        switch (sm.get()) {

            case ATIRANDO_FRENTE -> {

                double alvo = rpmAlvo;

                if (alimentando) {
                    alvo += ConstantesShooter.RPM_ANTI_DROP;
                }

                if (Double.isNaN(ultimoSetpoint) || ultimoSetpoint != alvo) {
                    io.arlindopid.setSetpoint(alvo, ControlType.kVelocity);
                    ultimoSetpoint = alvo;
                }
            }

            case ATIRANDO_TRAS -> {
                double alvo = -rpmAlvo;

                if (Double.isNaN(ultimoSetpoint) || ultimoSetpoint != alvo) {
                    io.arlindopid.setSetpoint(alvo, ControlType.kVelocity);
                    ultimoSetpoint = alvo;
                }
            }

            case PARADO -> {
                io.arlindo.stopMotor();
                ultimoSetpoint = Double.NaN;
                alimentando = false;
            }
        }

        SmartDashboard.putString("Shooter/Estado", sm.get().name());
        SmartDashboard.putNumber("Shooter/RPM Alvo", rpmAlvo);
        SmartDashboard.putNumber("Shooter/RPM Bruto", rpmBruto);
        SmartDashboard.putNumber("Shooter/RPM Filtrado", rpmFiltradoDashboard);
        SmartDashboard.putBoolean("Shooter/Pronto", pronto());
    }
}
