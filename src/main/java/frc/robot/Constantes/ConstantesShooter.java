package frc.robot.Constantes;

public class ConstantesShooter {

    public enum Velocidade {
        NORMAL(2400),
        MEDIA(3350),
        ALTA(3500),
        TURBO(3750);

        public final double rpm;

        Velocidade(double rpm) {
            this.rpm = rpm;
        }
    }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class Shooter{
   public static final int ShooterArlindo =10;
     public static final int ShooterBoquinha =11;
  }
  public static class EncoderShooter{
    public static final int ArlindoEncoder = 13;
    public static final int BoquinhaEncoder = 12;
  }

    public static final double TOLERANCIA_RPM = 100;
    public static final double TOLERANCIA_RPM_SAIDA = 150;
    public static final double RAMP_RATE_S = 0.05;
    public static final double RPM_MAXIMO_CONTROLE = 5700.0;
    public static final int CORRENTE_MAXIMA_A = 60;
    
    public static final double  PID_P = 0.0004;
    public static final double  PID_I = 0.0;
    public static final double  PID_D = 0.0;
    public static final double PID_IZONE_RPM = 250.0;
    public static final double FF_VELOCIDADE = 1.0 / 5700.0;
    public static final double boquinhaReducao = 25.0;
    public static final double arlindoReducao = 9.0;

    private ConstantesShooter() {}
}
