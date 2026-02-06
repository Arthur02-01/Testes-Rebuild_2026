package frc.robot.Constantes;

public class ConstantesShooter {

    public enum Velocidade {
        NORMAL(2400),
        MEDIA(3500),
        ALTA(4200),
        TURBO(5700);

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
    public static final double RAMP_RATE_S = 0.2;
    public static final double boquinhaReducao = 25.0;
    public static final double arlindoReducao = 9.0;

    private ConstantesShooter() {}
}
