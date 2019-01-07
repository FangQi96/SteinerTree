package Tester;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;
import steiner.model.Vertex;

import java.util.ArrayList;

public class SimpleWeightedGraphTest {
    public static void main(String[] args0){
        Graph<Vertex,DefaultWeightedEdge> g = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

        Vertex v1 = new Vertex(1);
        Vertex v2 = new Vertex(2);
        Vertex v3 = new Vertex(3);
        Vertex v4 = new Vertex(4);

        DefaultWeightedEdge edge1 = new DefaultWeightedEdge();

        g.addVertex(v1);
        g.addVertex(v2);
        g.addVertex(v3);
        g.addVertex(v4);

        g.addEdge(v1,v2);
        g.addEdge(v1,v3);
        g.addEdge(v1,v4);

        System.out.println();
    }
}

