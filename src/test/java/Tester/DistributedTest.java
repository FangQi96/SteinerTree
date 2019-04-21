package Tester;
import steiner.algo.Steiner_Distributed;
import steiner.model.SteinerGraph;

import java.util.*;


public class DistributedTest{
    public static final int nodeNum= 400;
    public static final int sourceNum = 5;

    public static void main(String[] args0) throws Exception {
        double[][] adjmatrix = new double[nodeNum][nodeNum];
        ArrayList<Integer> sources = new ArrayList<>();
        //randomGenerateSource(sourceNum);     //randomly generate sources
        GraphInitializerTest.readFile(adjmatrix,sources,"400nodes.txt","p2psrc.txt");

        SteinerGraph graph = new SteinerGraph(adjmatrix, sources);
        Steiner_Distributed steiner_distributed = new Steiner_Distributed(graph,1.3,0.91,0.01/400);
        steiner_distributed.globalPreCalculation(50);
        steiner_distributed.vertexInitial();
        steiner_distributed.distributedIteration(2,50);
        steiner_distributed.visualization();
   }
}