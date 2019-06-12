package Tester;
import steiner.algo.Steiner_Distributed;
import steiner.model.SteinerGraph;
import steiner.model.Vertex;

import java.util.*;

import static Tester.GraphInitializerTest.randomGenerateSource;


public class DistributedTest{
    public static final int nodeNum= 100;
    public static final int sourceNum = 5;

    public static void main(String[] args0) throws Exception {
        double[][] adjmatrix = new double[nodeNum][nodeNum];
        ArrayList<Integer> sources = new ArrayList<>();
        randomGenerateSource(nodeNum,sourceNum);     //randomly generate sources
        GraphInitializerTest.readFile(adjmatrix,sources,"test111.txt","p2psrc.txt",sourceNum);


        SteinerGraph graph = new SteinerGraph(adjmatrix, sources);
        Steiner_Distributed steiner_distributed = new Steiner_Distributed(graph,1.3,0.88,0.01/100);
        steiner_distributed.globalPreCalculation(15);
        steiner_distributed.vertexInitial();

        for(Vertex vertex:steiner_distributed.getSteiner_global().getGraph().getGraph().vertexSet()){
            while(!vertex.isCompleted())
                Thread.sleep(1);
        }

        steiner_distributed.visualization();
   }
}