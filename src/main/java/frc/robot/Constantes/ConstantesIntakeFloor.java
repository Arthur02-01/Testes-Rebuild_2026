package frc.robot.Constantes;

public class ConstantesIntakeFloor {
    public static final class IntakeFloorMotor{
      public static final int IntakeMotor = 7;
  }
    public static final class PivotMotor{
      public static final int PivotMotor = 8;
  }

  public static class EncoderPivot{
    public static final int PivotAngulador = 44;
  }

  public static class FFPivot{
    public static final double kS = 0.13;
    public static final double kG = 0.40;
    public static final double kV = 1.0;
}
  public static final double LIMITE_INFERIOR_PIVOT = 5.0;
  public static final double LIMITE_SUPERIOR_PIVOT = 120.0;

  public static final double REDUCAO_PIVOT = 25.0;

  public static final double DT_PIVOT = 0.02;
  public static final double MARGEM_ERRO_BASE_PIVOT = 1.70;

  public static final double VELOCIDADE_MAX_PIVOT = 0.25;
  public static final double VELOCIDADE_MAX_PIVOT_NEGATIVA = -0.25;
  
  public static final double MAX_VEL_PIVOT = 90.0;
  public static final double MAX_ACC_PIVOT = 180.0;

  public static final double VELOCIDADE_MIN_PIVOT = 0.01;
  public static final double TEMPO_MAX_TRAVADO_PIVOT = 0.8;
  public static final double ERRO_TOLERANCIA_PIVOT = 2.0;

  private ConstantesIntakeFloor () {}
}
