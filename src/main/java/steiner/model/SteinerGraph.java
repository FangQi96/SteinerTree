package steiner.model;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import java.io.IOException;
import java.util.*;

import static java.lang.Math.abs;
import static org.jgrapht.Graphs.addEdgeWithVertices;
/******************************************************
 *this class is just a shell to implement more methods*
 ******************************************************/
public class SteinerGraph extends SimpleWeightedGraph{
    private int vertex_num;
    private int source_num;

    public ArrayList<Integer> getSourceList() {
        return source_list;
    }

    private ArrayList<Integer> source_list = new ArrayList<>();

    public ArrayList<double[]> getPressure() {
        return pressure;
    }

    public double[][] getAdjmatrix() {
        return adjmatrix;
    }

    private double[][] adjmatrix;
    private ArrayList<double[]> pressure;

    public Graph<Vertex, Edge> getGraph() {
        return graph;
    }

    private Graph<Vertex,Edge> graph;

    /*
    public void printPressure(){
        for(double[] vector:pressure){
            for(int i=0;i<vector.length;i++){
                if(i!=vector.length)
                    System.out.printf("%.3f ",vector[i]);
                else
                    System.out.printf("%.3f\n",vector[i]);
            }
            System.out.println();
        }
    }
    */

    /*******************************************************************
     TODO***************************************************************
    /****Vertex can be duplicate initialized and it's a little tricky to
     **solve because JGraphT's API doesn't allow to add edges in loops.
     *******************************************************************/

    public SteinerGraph(double[][] adjmatrix, Collection<Integer> source) throws IOException {      //initialize a graph with 2d-dimension square matrix
        super(DefaultWeightedEdge.class);
        this.adjmatrix = adjmatrix;                 //Exception needed here
        vertex_num = adjmatrix[0].length;
        source_num = source.size();
        pressure = new ArrayList<>();
        this.graph = new SimpleWeightedGraph<>(Edge.class);

        HashMap<Integer,Vertex> vertexMap = new HashMap<>();

        for(int i=0;i<vertex_num;i++){
            if(source.contains(i)) {
                Vertex v = new Vertex(i,true);
                graph.addVertex(v);
                vertexMap.put(i,v);
            }
            else {
                Vertex v = new Vertex(i,false);
                graph.addVertex(v);
                vertexMap.put(i,v);
            }
        }//add vertices
        for(int i=0;i<vertex_num;i++) {        //Only iterate upper half of the matrix
            for (int j = i + 1; j < vertex_num; j++) {
                if (abs(adjmatrix[i][j]) > 0.0001) {
                    Vertex vertex_i = vertexMap.get(i);
                    Vertex vertex_j = vertexMap.get(j);
                    addEdgeWithVertices(graph, vertex_i, vertex_j, adjmatrix[i][j]);
                }
            }
        }
    }

    public double getEdgeWeightSum(){
        Set<Edge> edges = this.graph.edgeSet();
        double sum = 0;
        for(Edge e:edges){
            sum += e.getWeight();
        }
        return sum;
    }

    public int getSourceNum() {
        return source_num;
    }
}
