package Tester;
import org.apache.commons.math3.linear.*;

public class MatrixSolverTest {
    public static void main(String[] args0) {
        RealMatrix coefficients = new Array2DRowRealMatrix(new double[][]{{3,-1,-1,-1}, {-1, 1, 0, 0},
                {1,0,0,0}, {-1,0,0,1}}, false);
        DecompositionSolver solver = new QRDecomposition(coefficients).getSolver();
        RealVector constants = new ArrayRealVector(new double[]{0,1,1,0}, false);
        RealVector solution = solver.solve(constants);
        double[] out = solution.toArray();
        for(int i=0;i<out.length;i++){
            System.out.println(out[i]+" ");
        }
    }
}
