package steiner.algo;

import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import steiner.model.Edge;
import steiner.model.SteinerGraph;
import steiner.model.Vertex;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.*;


public class ShortestPathTree {
    private SteinerGraph steinerGraph;

    public ShortestPathTree(SteinerGraph steinerGraphgraph){
        this.steinerGraph = steinerGraphgraph;
    }

    public Set<Edge> getShortestPathTree() {
        Set<Edge> SPT = new HashSet<>();
        Set<Vertex> sources = new HashSet<>();
        Vertex root = null;
        for(Vertex v : this.steinerGraph.getGraph().vertexSet()){
            if(v.isSource()) {
                root = v;
                break;
            }
        }
        for(Vertex v : this.steinerGraph.getGraph().vertexSet()){
            if(v.isSource())
                sources.add(v);
        }
        for(Vertex source: sources){
            if(source.getName() != root.getName()){
                SPT.addAll(DijkstraShortestPath.findPathBetween(steinerGraph.getGraph(), root, source).getEdgeList());
            }
        }
        return SPT;
    }

    public double getShortestPathTreeWeight() {
        double sum = 0;
        Set<Edge> set = this.getShortestPathTree();
        for(Edge e: set){
            sum += e.getWeight();
        }
        return sum;
    }

    public static void readFile(double[][] adjmatrix,List<Integer> sources,String graph,String source,int sourceNum){
        File file = new File("/home/longinus/Documents/streetest/" + graph);     //file of the network
        Scanner sc = null;
        try {
            sc = new Scanner(file);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        int col,row;
        double cost;
        while(sc.hasNext()){
            col = sc.nextInt() - 1;
            row = sc.nextInt() - 1;
            cost = sc.nextDouble() / 100;
            adjmatrix[col][row]  = adjmatrix[row][col] = cost;
        }


        File file_source = new File("/home/longinus/Documents/streetest/" + source);       //file of the sources
        try {
            Scanner sc1 = new Scanner(file_source);
            for(int i=0;i<sourceNum;i++){
                sources.add(sc1.nextInt() - 1);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

    }

    public static void main(String[] args0) throws IOException {
        int nodeNum = 400;
        int sourceNum = 30;
        double[][] adjmatrix = new double[nodeNum][nodeNum];
        List<Integer> sources = new ArrayList<>();
        readFile(adjmatrix,sources,"400nodes_dense.txt","p2psrc.txt",sourceNum);

        SteinerGraph graph = new SteinerGraph(adjmatrix, sources);
        ShortestPathTree SPT = new ShortestPathTree(graph);
        System.out.println("Sum: " + SPT.getShortestPathTreeWeight());
    }
}
