package Tester;
import steiner.algo.Steiner_Global;
import steiner.model.Edge;
import steiner.model.SteinerGraph;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
/*********************************/
/* the weight of the edge, AKA C_ij has a great influence on the result,
adjusting rho in Steiner_Global.java to adapt to a specific given graph

 */
/*********************************/
public class GraphInitializerTest {
    public static void main(String[] args0){
        double[][] adjmatrix = {
                {0,1,0,1,0,0,0,0,0},
                {1,0,9,0,9,0,0,0,0},
                {0,9,0,0,0,2,0,0,0},
                {1,0,0,0,9,0,1,0,0},
                {0,9,0,9,0,1,0,1,0},
                {0,0,2,0,1,0,0,0,1},
                {0,0,0,1,0,0,0,1,0},
                {0,0,0,0,1,0,1,0,1},
                {0,0,0,0,0,1,0,1,0}
        };

        List<Integer> list = new ArrayList<>();
        list.add(0);
        list.add(3);
        list.add(6);
        SteinerGraph graph = new SteinerGraph(adjmatrix,list);
//        Iterator<Vertex> iterator = graph.getGraph().vertexSet().iterator();
//        Vertex v1 = iterator.next();
//        Vertex v2 = iterator.next();
//        Edge edge = graph.getGraph().getEdge(v1,v2);
        Steiner_Global global = new Steiner_Global(graph);
        global.initial();
        global.iteration(10000);
        global.visualization();
        Set<Edge> edgeSet = global.getGraph().getGraph().edgeSet();
        System.out.println(1);
    }
}
