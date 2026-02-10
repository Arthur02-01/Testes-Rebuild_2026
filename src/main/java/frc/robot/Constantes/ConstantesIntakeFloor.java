package frc.robot.Constantes;

public class ConstantesIntakeFloor {

    public static final class IntakeFloorMotor {
        public static final int IntakeMotor = 7;
    }

    public static final class PivotMotor {
        public static final int PivotMotor = 8;
    }

    // ======================
    // CINEMÁTICA
    // ======================
    public static final double REDUCAO_PIVOT = 25.0;

    // Zero físico: braço APONTANDO PARA BAIXO
    public static final double OFFSET_PIVOT_RAD = Math.toRadians(0.0);

    // ======================
    // LIMITES MECÂNICOS
    // ======================
    public static final double LIMITE_INFERIOR_PIVOT = Math.toRadians(15.0);
    public static final double LIMITE_SUPERIOR_PIVOT = Math.toRadians(150.0);

    // ======================
    // PERFIL TRAPEZOIDAL
    // ======================
    public static final double DT_PIVOT = 0.02;
    public static final double MAX_VEL_PIVOT = Math.toRadians(90.0);   // rad/s
    public static final double MAX_ACC_PIVOT = Math.toRadians(180.0);  // rad/s²

    // ======================
    // TOLERÂNCIAS
    // ======================
    public static final double ERRO_TOLERANCIA_PIVOT = Math.toRadians(2.5);
    public static final double MARGEM_ERRO_BASE_PIVOT = Math.toRadians(1.5);

    // ======================
    // FEEDFORWARD
    // ======================
    public static final class FFPivot {
        public static final double kS = 0.65;
        public static final double kG = 0.85;   // AJUSTE FINO DEPOIS
        public static final double kV = 0.2;
    }

    private ConstantesIntakeFloor() {}
}
