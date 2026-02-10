package frc.robot.subsystems;

import frc.robot.StatesMachines.StateMachineIndex;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constantes.ConstantesIndex;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Hardwares.HardwaresIndex;

public class Index extends SubsystemBase {

   
    private final HardwaresIndex io = new HardwaresIndex();
    private final StateMachineIndex sm = new StateMachineIndex();

    private double rpmAlvo = ConstantesShooter.Velocidade.NORMAL.rpm;
    private double ultimoSetpointBoquinha = Double.NaN;

    private double setpointAplicadoRpm = 0.0;
    private double ultimaAtualizacaoSetpointS = 0.0;

    private boolean pronto = false;
    private double entrouNaFaixaEm = -1.0;

    private double tempoInicioDirecionar = -1.0;

    private final LinearFilter filtroRpmBoquinha =
        LinearFilter.singlePoleIIR(
            ConstantesShooter.FILTRO_RPM_TAU_S,
            0.02
        );

    private double rpmBoquinhaFiltradoAtual = 0.0;

    private ConstantesIndex.VelocidadeIndex velocidadeIndex =
        ConstantesIndex.VelocidadeIndex.NORMAL;

    public void setVelocidade(ConstantesIndex.VelocidadeIndex vel) {
        this.velocidadeIndex = vel;
        this.rpmAlvo = vel.rpm;
    }
    public void ligar() {
        sm.set(StateMachineIndex.Estado.DIRECIONAR);
    }

    public void desligar() {
        sm.set(StateMachineIndex.Estado.SOSSEGADO);
    }

    public boolean estaLigado() {
        return sm.is(StateMachineIndex.Estado.DIRECIONAR);
    }

    /* ================= BOQUINHA DIRETA (SEM INDEX) ================= */

    public void ligarBoquinha(double rpm) {
        this.rpmAlvo = rpm;
        sm.set(StateMachineIndex.Estado.DIRECIONAR);
    }

    public void desligarBoquinha() {
        sm.set(StateMachineIndex.Estado.SOSSEGADO);
    }


    public boolean pronto() {

        double erro = Math.abs(rpmAlvo - rpmBoquinhaFiltradoAtual);
        double agora = Timer.getFPGATimestamp();

        if (erro < ConstantesIndex.TOLERANCIA_RPM_01) {
            if (entrouNaFaixaEm < 0.0) {
                entrouNaFaixaEm = agora;
            }
        } else if (erro > ConstantesIndex.TOLERANCIA_RPM_SAIDA_01) {
            entrouNaFaixaEm = -1.0;
            pronto = false;
            return false;
        }

        if (entrouNaFaixaEm > 0.0 &&
            (agora - entrouNaFaixaEm) >= ConstantesIndex.TEMPO_ESTABILIZACAO_S_01) {
            pronto = true;
        }

        return pronto;
    }


    private double calcularSetpointSuave(double rpmDesejado) {

        double agora = Timer.getFPGATimestamp();

        if (ultimaAtualizacaoSetpointS <= 0.0) {
            ultimaAtualizacaoSetpointS = agora;
            setpointAplicadoRpm = rpmDesejado;
            return setpointAplicadoRpm;
        }

        double dt = Math.max(0.0, agora - ultimaAtualizacaoSetpointS);
        double passoMax =
            ConstantesIndex.TAXA_RAMPA_SETPOINT_RPM_POR_S_01 * dt;

        double erro = rpmDesejado - setpointAplicadoRpm;

        erro = Math.copySign(
            Math.min(Math.abs(erro), passoMax),
            erro
        );

        setpointAplicadoRpm += erro;
        ultimaAtualizacaoSetpointS = agora;

        return setpointAplicadoRpm;
    }

    private void definirSetpointBoquinha(double rpm) {

        double limitado = Math.copySign(
            Math.min(Math.abs(rpm), ConstantesShooter.RPM_MAXIMO_CONTROLE),
            rpm
        );

        if (Double.isNaN(ultimoSetpointBoquinha)
            || ultimoSetpointBoquinha != limitado) {

            io.boquinhapid.setSetpoint(limitado, ControlType.kVelocity);
            ultimoSetpointBoquinha = limitado;
        }
    }


    @Override
    public void periodic() {

        double rpmBoquinhaBruto = io.boquinhaEncoder.getVelocity();
        rpmBoquinhaFiltradoAtual =
            Math.abs(filtroRpmBoquinha.calculate(rpmBoquinhaBruto));

        switch (sm.get()) {

            case DIRECIONAR -> {

                double agora = Timer.getFPGATimestamp();

                if (tempoInicioDirecionar < 0.0) {
                    tempoInicioDirecionar = agora;
                }

                double rpmSuavizado = calcularSetpointSuave(rpmAlvo);
                definirSetpointBoquinha(rpmSuavizado);

                if ((agora - tempoInicioDirecionar)
                        >= ConstantesIndex.ATRASO_INDEX_S) {

                    io.index.set(velocidadeIndex.percentual);

                } else {
                    io.index.stopMotor();
                }
            }

            case SOSSEGADO -> {

                io.boquinha.stopMotor();
                io.index.stopMotor();

                tempoInicioDirecionar = -1.0;
                ultimoSetpointBoquinha = Double.NaN;

                pronto = false;
                entrouNaFaixaEm = -1.0;
                setpointAplicadoRpm = 0.0;
                ultimaAtualizacaoSetpointS = 0.0;
            }
        }


        double agora = Timer.getFPGATimestamp();
        SmartDashboard.putString("Index/Estado", sm.get().name());
        SmartDashboard.putNumber("Index/RPM Alvo", rpmAlvo);
        SmartDashboard.putNumber("Index/RPM Boquinha", rpmBoquinhaFiltradoAtual);
        SmartDashboard.putBoolean("Index/Pronto", pronto());
    }
}
