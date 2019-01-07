package Tester;
import steiner.algo.Steiner_Global;
import steiner.model.SteinerGraph;
import steiner.model.Vertex;

import java.beans.VetoableChangeListener;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class GraphInitializerTest {
    public static void main(String[] args0){
        double[][] adjmatrix = {
                {0,1},
                {1,0},
        };
        List<Integer> list = new ArrayList<>();
        list.add(0);
        list.add(1);
        SteinerGraph graph = new SteinerGraph(adjmatrix,list);
        Steiner_Global global = new Steiner_Global(graph);
        global.initial();
        global.iteration(10);
        System.out.println(1);
    }
}
