package frc.robot.Kinematics;

import frc.robot.Constantes.ConstantesIntakeFloor;

public class KinematicsIntakeFloor {
        
    public static double grausParaRotacoesPivot(double graus) {
        return (graus / 360.0) * ConstantesIntakeFloor.REDUCAO_PIVOT;
    }

    public static double rotacoesParaGrausPivot(double rotacoes) {
        return rotacoes * (360.0 / ConstantesIntakeFloor.REDUCAO_PIVOT);
    }

}
