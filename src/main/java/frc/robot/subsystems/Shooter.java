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
    private double ultimoSetpointArlindo = Double.NaN;
    private boolean pronto = false;
    private double entrouNaFaixaEm = -1.0;
    private double setpointAplicadoRpm = 0.0;
    private double ultimaAtualizacaoSetpointS = 0.0;
    private double ultimoDashboard = 0.0;

    private final LinearFilter filtroRpmArlindo =
        LinearFilter.singlePoleIIR(ConstantesShooter.FILTRO_RPM_TAU_S, 0.02);

    private double rpmArlindoFiltradoAtual = 0.0;


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
            double rpmArlindo = rpmArlindoFiltradoAtual;

        boolean dentro =
            Math.abs(rpmAlvo - rpmArlindo) < ConstantesShooter.TOLERANCIA_RPM;

        boolean fora =
             Math.abs(rpmAlvo - rpmArlindo) > ConstantesShooter.TOLERANCIA_RPM_SAIDA;

        double agora = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        return pronto;
    }

     private double calcularSetpointSuavizado(double rpmDesejado) {
        double agora = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (ultimaAtualizacaoSetpointS <= 0.0) {
            ultimaAtualizacaoSetpointS = agora;
            setpointAplicadoRpm = rpmDesejado;
            return setpointAplicadoRpm;
        }

        double dt = Math.max(0.0, agora - ultimaAtualizacaoSetpointS);
        double passoMaximo = ConstantesShooter.TAXA_RAMPA_SETPOINT_RPM_POR_S * dt;
        double erro = rpmDesejado - setpointAplicadoRpm;

        if (erro > passoMaximo) {
            erro = passoMaximo;
        } else if (erro < -passoMaximo) {
            erro = -passoMaximo;
        }

        setpointAplicadoRpm += erro;
        ultimaAtualizacaoSetpointS = agora;
        return setpointAplicadoRpm;
    }


    private void definirSetpoint(double arlindoRpm) {

    double limitado = Math.copySign(
        Math.min(Math.abs(arlindoRpm), ConstantesShooter.RPM_MAXIMO_CONTROLE),
        arlindoRpm
    );

    if (Double.isNaN(ultimoSetpointArlindo)
        || ultimoSetpointArlindo != limitado) {

        io.arlindopid.setSetpoint(limitado, ControlType.kVelocity);
        ultimoSetpointArlindo = limitado;
    }
}

    
        
    

    /* ================= LOOP ================= */

    @Override
    public void periodic() {

        double rpmArlindoBruto = io.arlindoEncoder.getVelocity();
        rpmArlindoFiltradoAtual = Math.abs(filtroRpmArlindo.calculate(rpmArlindoBruto));


        switch (sm.get()) {

            case ATIRANDO_FRENTE -> {
                double rpmSuavizado = calcularSetpointSuavizado(+rpmAlvo);
                definirSetpoint(rpmSuavizado);
            }

            case ATIRANDO_TRAS -> {
                double rpmSuavizado = calcularSetpointSuavizado(-rpmAlvo);
                definirSetpoint(rpmSuavizado);
            }

            case PARADO -> {
                io.arlindo.stopMotor();
                ultimoSetpointArlindo = Double.NaN;
                pronto = false;
                entrouNaFaixaEm = -1.0;
                setpointAplicadoRpm = 0.0;
                ultimaAtualizacaoSetpointS = 0.0;
                rpmArlindoFiltradoAtual = 0.0;
            }
        }
        double agora = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
         if (agora - ultimoDashboard >= 0.2) {
        ultimoDashboard = agora;
            SmartDashboard.putString("Shooter/Estado", sm.get().name());
            SmartDashboard.putNumber("Shooter/RPM Alvo", rpmAlvo);
            SmartDashboard.putNumber("Shooter/Setpoint Aplicado RPM", setpointAplicadoRpm);
            SmartDashboard.putNumber("Shooter/Arlindo RPM", rpmArlindoBruto);
            SmartDashboard.putNumber("Shooter/Arlindo RPM Filtrado", rpmArlindoFiltradoAtual);
            SmartDashboard.putNumber("Shooter/Erro Arlindo RPM", rpmAlvo - rpmArlindoFiltradoAtual);
            SmartDashboard.putBoolean("Shooter/Pronto", pronto());
        }
    }
}
