package frc.robot.Constantes;

import frc.robot.subsystems.Climber;

public class ConstantesClimber {
        /* ================== CONSTANTES FÍSICAS ================== */

    public static final double DIAMETRO_TAMBOR = 0.30;               // Estão em Metros todas
    public static final double CIRCUNFERENCIA = Math.PI * DIAMETRO_TAMBOR;
    public static final double REDUCAO = 64.0;

    public static final double LIMITE_INFERIOR = 0.0;                
    public static final double LIMITE_SUPERIOR = 1.15;              

    public static final double MARGEM_ERRO = 0.01;                  
    public static final double ERRO_TRAVAMENTO = 0.02;
    


    /* ================== DETECÇÃO DE TRAVAMENTO ================== */

    public static final double DT = 0.02;
    public static final double VELOCIDADE_MIN = 0.002;               
    public static final double TEMPO_MAX_TRAVADO = 0.4;              

    public double posicaoAnterior = 0.0;
    public double tempoSemMovimento = 0.0;

    private ConstantesClimber() {}
}
