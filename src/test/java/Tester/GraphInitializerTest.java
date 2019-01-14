package Tester;
import org.jgrapht.Graph;
import org.jgrapht.generate.ScaleFreeGraphGenerator;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;
import steiner.algo.Steiner_Global;
import steiner.model.Edge;
import steiner.model.SteinerGraph;
import steiner.model.Vertex;

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
                {0, 6, 1, 0, 0, 0, 0, 0, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {6, 0, 2, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {1, 2, 0, 2, 0, 0, 8, 1, 0, 0, 0, 1, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0},
                {0, 1, 2, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 4, 0, 0, 0, 4, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 1, 4, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 8, 3, 0, 0, 0, 0, 1, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 1, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 3, 0, 0, 0, 1, 0, 0, 0},
                {0, 0, 0, 0, 2, 2, 1, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 6, 8, 3, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 1, 0},
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {3, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 1, 0, 1, 0, 0, 3, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 3, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0, 0},
                {0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 3},
                {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 5, 0, 1},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 5, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 1, 0, 3, 0}
        };

        double sum = 0;
        int count = 0;
        for(int i=0;i<22;i++){
            for(int j=0;j<22;j++){
                if((adjmatrix[i][j]-0)>0.0001){
                    sum += adjmatrix[i][j];
                    count++;
                }
            }
        }
        System.out.println(sum/count);
        System.out.println();

        List<Integer> list = new ArrayList<>();
        list.add(2);
        list.add(4);
        list.add(8);
        list.add(13);
        list.add(17);
        list.add(20);
        SteinerGraph graph = new SteinerGraph(adjmatrix,list);
        Steiner_Global global = new Steiner_Global(graph);
        global.initial();
        global.setDelta();
        global.iteration(11000);
        Set<Edge> edgeSet = global.getGraph().getGraph().edgeSet();
        global.cutEdge();
        global.visualization();

/*        ScaleFreeGraphGenerator<Integer,DefaultWeightedEdge> generator = new ScaleFreeGraphGenerator<>(10);
        SimpleWeightedGraph<Integer,DefaultWeightedEdge> target = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);
        generator.generateGraph(target);
        SteinerGraph steinerGraph = new SteinerGraph(target);
        Steiner_Global test = new Steiner_Global(steinerGraph);
        test.visualization();*/
    }
}
