package Tester;

import java.util.Random;

public class AdjacencyMatrixGenerator {
    public static void main(String[] args){
        double[][] adjmatrix = {
                {0.00, 5.39, 9.75, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 6.16, 7.37, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {5.39, 0.00, 2.36, 3.68, 7.37, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {9.75, 2.36, 0.00, 4.89, 0.00, 0.00, 4.89, 6.02, 0.00, 0.00, 0.00, 6.01, 0.00, 0.00, 0.00, 2.98, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 3.68, 4.89, 0.00, 0.00, 7.36, 2.39, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 7.37, 0.00, 0.00, 0.00, 6.82, 0.00, 0.00, 8.61, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 0.00, 0.00, 7.36, 6.82, 0.00, 0.00, 0.00, 8.65, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 0.00, 4.89, 2.39, 0.00, 0.00, 0.00, 0.00, 9.86, 9.32, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 0.00, 6.02, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 5.79, 0.00, 0.00, 0.00, 0.00, 2.57, 0.00, 0.00, 0.00, 2.25, 0.00, 0.00, 0.00},
                {0.00, 0.00, 0.00, 0.00, 8.61, 8.65, 9.86, 0.00, 0.00, 3.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 9.32, 5.79, 3.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 6.84, 0.00, 0.00, 3.02, 0.00},
                {6.16, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 6.51, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {7.37, 0.00, 6.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 6.24, 0.00, 0.00, 4.13, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 6.51, 6.24, 0.00, 4.27, 0.00, 0.00, 3.87, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 4.27, 0.00, 0.00, 6.17, 8.89, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.57, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 8.00, 0.00, 0.00, 0.00, 7.53, 0.00, 0.00},
                {0.00, 0.00, 2.98, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 4.13, 0.00, 6.17, 8.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 3.87, 8.89, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
                {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 6.84, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.06, 0.00, 0.00, 6.30},
                {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.25, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.06, 0.00, 7.36, 0.00, 1.93},
                {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 7.53, 0.00, 0.00, 0.00, 7.36, 0.00, 0.00, 0.00},
                {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 3.02, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 6.12},
                {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 6.30, 1.93, 0.00, 6.12, 0.00},
        };
        double[][] rand = new double[adjmatrix[0].length][adjmatrix[0].length];
        for(int i=0;i<adjmatrix[0].length;i++){
            for(int j=i+1;j<adjmatrix[0].length;j++){
                if(adjmatrix[i][j]>0.01){
                    rand[i][j] = Math.random() * 9 + 1;
                }
            }
        }
        for(int i=0;i<adjmatrix[0].length;i++){
            for(int j=0;j<adjmatrix[0].length;j++){
                rand[j][i] = rand[i][j];
            }
        }
        for(int i=0;i<rand[0].length;i++){
            for(int j=0;j<rand[0].length;j++){
                if(j==0)
                    System.out.print('{');
                System.out.printf("%.2f",rand[i][j]);
                if(j==rand[0].length-1)
                    System.out.print('}');
                System.out.print(", ");
            }
            System.out.println();
        }
    }
}