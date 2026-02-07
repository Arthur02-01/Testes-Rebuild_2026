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
    private double ultimoSetpointBoquinha = Double.NaN;
    private boolean pronto = false;
    private double entrouNaFaixaEm = -1.0;
    private double setpointAplicadoRpm = 0.0;
    private double ultimaAtualizacaoSetpointS = 0.0;
    private double ultimoDashboard = 0.0;

    private final LinearFilter filtroRpmArlindo =
        LinearFilter.singlePoleIIR(ConstantesShooter.FILTRO_RPM_TAU_S, 0.02);
    private final LinearFilter filtroRpmBoquinha =
        LinearFilter.singlePoleIIR(ConstantesShooter.FILTRO_RPM_TAU_S, 0.02);

    private double rpmArlindoFiltradoAtual = 0.0;
    private double rpmBoquinhaFiltradoAtual = 0.0;

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
        double rpmBoquinha = rpmBoquinhaFiltradoAtual;

        boolean dentro =
            Math.abs(rpmAlvo - rpmArlindo) < ConstantesShooter.TOLERANCIA_RPM
            && Math.abs(rpmAlvo - rpmBoquinha) < ConstantesShooter.TOLERANCIA_RPM;

        boolean fora =
             Math.abs(rpmAlvo - rpmArlindo) > ConstantesShooter.TOLERANCIA_RPM_SAIDA
            || Math.abs(rpmAlvo - rpmBoquinha) > ConstantesShooter.TOLERANCIA_RPM_SAIDA;

            boolean motoresBalanceados =
            Math.abs(rpmArlindo - rpmBoquinha)
                <= ConstantesShooter.DELTA_MAXIMO_ENTRE_MOTORES_RPM;

        double agora = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (dentro && motoresBalanceados) {
            if (entrouNaFaixaEm < 0.0) {
                entrouNaFaixaEm = agora;
            }
            if (agora - entrouNaFaixaEm >= ConstantesShooter.TEMPO_ESTABILIZACAO_S) {
                pronto = true;
            }
        } else if (fora || !motoresBalanceados) {
            pronto = false;
            entrouNaFaixaEm = -1.0;
        }

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


    private void definirSetpoint(double arlindoRpm, double boquinhaRpm) {
            
        double arlindoLimitado = Math.copySign(
            Math.min(Math.abs(arlindoRpm), ConstantesShooter.RPM_MAXIMO_CONTROLE),
            arlindoRpm
        );

        double boquinhaLimitado = Math.copySign(
            Math.min(Math.abs(boquinhaRpm), ConstantesShooter.RPM_MAXIMO_CONTROLE),
            boquinhaRpm
        );

        if (Double.isNaN(ultimoSetpointArlindo)
               || ultimoSetpointArlindo != arlindoLimitado) {
            io.arlindopid.setSetpoint(arlindoLimitado, ControlType.kVelocity);
            ultimoSetpointArlindo = arlindoLimitado;
        }
        if (Double.isNaN(ultimoSetpointBoquinha)
               || ultimoSetpointBoquinha != boquinhaLimitado) {
            io.boquinhapid.setSetpoint(boquinhaLimitado, ControlType.kVelocity);
            ultimoSetpointBoquinha = boquinhaLimitado;
        }
    }
    
        
    

    /* ================= LOOP ================= */

    @Override
    public void periodic() {

        double rpmArlindoBruto = io.arlindoEncoder.getVelocity();
        double rpmBoquinhaBruto = io.boquinhaEncoder.getVelocity();
        rpmArlindoFiltradoAtual = Math.abs(filtroRpmArlindo.calculate(rpmArlindoBruto));
        rpmBoquinhaFiltradoAtual = Math.abs(filtroRpmBoquinha.calculate(rpmBoquinhaBruto));


        switch (sm.get()) {

            case ATIRANDO_FRENTE -> {
                double rpmSuavizado = calcularSetpointSuavizado(+rpmAlvo);
                definirSetpoint(rpmSuavizado, rpmSuavizado);
            }

            case ATIRANDO_TRAS -> {
                double rpmSuavizado = calcularSetpointSuavizado(-rpmAlvo);
                definirSetpoint(rpmSuavizado, rpmSuavizado);
            }

            case PARADO -> {
                io.arlindo.stopMotor();
                io.boquinha.stopMotor();
                ultimoSetpointArlindo = Double.NaN;
                ultimoSetpointBoquinha = Double.NaN;
                pronto = false;
                entrouNaFaixaEm = -1.0;
                setpointAplicadoRpm = 0.0;
                ultimaAtualizacaoSetpointS = 0.0;
                rpmArlindoFiltradoAtual = 0.0;
                rpmBoquinhaFiltradoAtual = 0.0;
            }
        }
        double agora = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
         if (agora - ultimoDashboard >= 0.2) {
        ultimoDashboard = agora;
            SmartDashboard.putString("Shooter/Estado", sm.get().name());
            SmartDashboard.putNumber("Shooter/RPM Alvo", rpmAlvo);
            SmartDashboard.putNumber("Shooter/Setpoint Aplicado RPM", setpointAplicadoRpm);
            SmartDashboard.putNumber("Shooter/Arlindo RPM", rpmArlindoBruto);
            SmartDashboard.putNumber("Shooter/Boquinha RPM", rpmBoquinhaBruto);
            SmartDashboard.putNumber("Shooter/Arlindo RPM Filtrado", rpmArlindoFiltradoAtual);
            SmartDashboard.putNumber("Shooter/Boquinha RPM Filtrado", rpmBoquinhaFiltradoAtual);
            SmartDashboard.putNumber("Shooter/Erro Arlindo RPM", rpmAlvo - rpmArlindoFiltradoAtual);
            SmartDashboard.putNumber("Shooter/Erro Boquinha RPM", rpmAlvo - rpmBoquinhaFiltradoAtual);
            SmartDashboard.putNumber("Shooter/Delta Entre Motores", Math.abs(rpmArlindoFiltradoAtual - rpmBoquinhaFiltradoAtual));
            SmartDashboard.putBoolean("Shooter/Pronto", pronto());
        }
    }
}
