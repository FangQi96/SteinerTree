package steiner.model;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;
import static org.jgrapht.Graphs.addEdgeWithVertices;
/*this class is just a shell to implement more methods*/
public class SteinerGraph extends SimpleWeightedGraph{
    private int vertex_num;
    private int source_num;

    public ArrayList<Integer> getSource_list() {
        return source_list;
    }

    private ArrayList<Integer> source_list = new ArrayList<>();

    public ArrayList<double[]> getPressure() {
        return pressure;
    }

    public void setPressure(ArrayList<double[]> pressure) {
        this.pressure = pressure;
    }

    public double[][] getAdjmatrix() {
        return adjmatrix;
    }

    //private int edge_num = 0;     //todo
    private double[][] adjmatrix;
    private ArrayList<double[]> pressure;

    public Graph<Vertex, Edge> getGraph() {
        return graph;
    }

    private Graph<Vertex,Edge> graph;

    public SteinerGraph(int vertex_num){        //initialize an empty steiner graph with n vertices
        super(DefaultWeightedEdge.class);
        this.vertex_num = vertex_num;
        this.graph = new SimpleWeightedGraph<>(Edge.class);
        for(int i = 0;i<vertex_num;i++){
            graph.addVertex(new Vertex(i));
        }
    }

    public SteinerGraph(double[][] adjmatrix, List<Integer> source){      //initialize a graph with 2d-dimension square matrix
        super(DefaultWeightedEdge.class);
        this.adjmatrix = adjmatrix;                 //Exception needed here
        vertex_num = adjmatrix[0].length;
        source_num = source.size();
        pressure = new ArrayList<>();
        this.graph = new SimpleWeightedGraph<>(Edge.class);
        for(int i=0;i<vertex_num-1;i++){
            for(int j=i+1;j<vertex_num;j++){
                if(abs(adjmatrix[i][j])>0.000001) {
                    Vertex vertex_i, vertex_j;
                    if(source.contains(i))
                        vertex_i = new Vertex(i,true);
                    else
                        vertex_i = new Vertex(i,false);
                    if(source.contains(j))
                        vertex_j = new Vertex(j,true);
                    else
                        vertex_j = new Vertex(j,false);
                    addEdgeWithVertices(graph, vertex_i, vertex_j, adjmatrix[i][j]);
                }
            }
        }
    }


    public int getSource_num() {
        return source_num;
    }
}
