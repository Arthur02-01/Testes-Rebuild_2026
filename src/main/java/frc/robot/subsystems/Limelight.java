package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final NetworkTable table;

    private double txFiltrado = 0.0;
    private double distFiltrada = 0.0;

    private static final double ALPHA = 0.2; 

    private static final double ALTURA_CAMERA = 0.65;   
    private static final double ALTURA_TAG    = 0.55;   
    private static final double ANGULO_CAMERA = 25.0;  

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean temAlvo() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return table.getEntry("ty").getDouble(0.0);
    }
    public double getTxFiltrado() {
        double tx = getTx();
        txFiltrado = (1 - ALPHA) * txFiltrado + ALPHA * tx;
        return txFiltrado;
    }

    public double getDistanciaAprilTag() {
        double ty = getTy();
        double anguloTotal = Math.toRadians(ANGULO_CAMERA + ty);

        double distancia =
            (ALTURA_TAG - ALTURA_CAMERA) / Math.tan(anguloTotal);

        return Math.abs(distancia);
    }

    public double getDistanciaFiltrada() {
        double d = getDistanciaAprilTag();
        distFiltrada = (1 - ALPHA) * distFiltrada + ALPHA * d;
        return distFiltrada;
    }

    public void ligarLED() {
        table.getEntry("ledMode").setNumber(3);
    }

    public void desligarLED() {
        table.getEntry("ledMode").setNumber(1);
    }

    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }
}
