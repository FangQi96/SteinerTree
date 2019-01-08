package Tester;
import steiner.algo.Steiner_Global;
import steiner.model.Edge;
import steiner.model.SteinerGraph;
import steiner.model.Vertex;

import java.beans.VetoableChangeListener;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

public class GraphInitializerTest {
    public static void main(String[] args0){
        double[][] adjmatrix = {
                {0,1,4,4},
                {1,0,1,1},
                {4,1,0,0},
                {4,1,0,0}
        };
        List<Integer> list = new ArrayList<>();
        list.add(0);
        list.add(1);
        SteinerGraph graph = new SteinerGraph(adjmatrix,list);
//        Iterator<Vertex> iterator = graph.getGraph().vertexSet().iterator();
//        Vertex v1 = iterator.next();
//        Vertex v2 = iterator.next();
//        Edge edge = graph.getGraph().getEdge(v1,v2);
        Steiner_Global global = new Steiner_Global(graph);
        global.initial();
        global.iteration(20000);
        System.out.println(1);
    }
}
