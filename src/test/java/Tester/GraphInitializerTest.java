package Tester;
import org.jgrapht.alg.connectivity.ConnectivityInspector;
import steiner.algo.Steiner_Global;
import steiner.model.Edge;
import steiner.model.SteinerGraph;
import steiner.model.Vertex;

import java.io.*;
import java.util.*;

import static org.apache.commons.lang3.RandomUtils.nextInt;
/*********************************/
/* the weight of the edge, AKA C_ij has a great influence on the result,
adjusting rho in Steiner_Global.java to adapt to a specific given graph

 */
/*********************************/
public class GraphInitializerTest {
    public static final int nodeNum= 441;
    public static final int sourceNum = 20;
    private static void randomGenerateSource(){
        File sources = new File("/home/longinus/Documents/SteinerTree/streetest/p2psrc.txt");
        try {
            sources.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            ArrayList<Integer> arrayList = new ArrayList<>();
            for(int i=0;i<nodeNum;i++){
                arrayList.add(i+1);
            }
            Collections.shuffle(arrayList);
            BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(sources));
            bufferedWriter.newLine();
            for(int i=0;i<sourceNum;i++){
                int sourceIndex = arrayList.get(i);
                bufferedWriter.write(String.valueOf(sourceIndex));
                bufferedWriter.newLine();
            }
            bufferedWriter.flush();
            bufferedWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
    public static void main(String[] args0){
        File file = new File("/home/longinus/Documents/SteinerTree/streetest/p2p.txt");     //file of the network
        Scanner sc = null;
        try {
            sc = new Scanner(file);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        double[][] adjmatrix = new double[nodeNum][nodeNum];
        int col,row;
        double cost;
        while(sc.hasNext()){
            col = sc.nextInt() - 1;
            row = sc.nextInt() - 1;
            cost = sc.nextDouble();
            adjmatrix[col][row] = cost;
        }

//        randomGenerateSource();     //randomly generate sources

        List<Integer> sources = new ArrayList<>();
        File file_source = new File("/home/longinus/Documents/SteinerTree/streetest/p2psrc.txt");       //file of the sources
        try {
            Scanner sc1 = new Scanner(file_source);
            for(int i=0;i<sourceNum;i++){
                sources.add(sc1.nextInt() - 1);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        SteinerGraph graph = new SteinerGraph(adjmatrix,sources);
        Steiner_Global global = new Steiner_Global(graph);
        global.initial();
//        global.setDelta();
        global.iteration(14000);
//        Set<Edge> edgeSet = global.getGraph().getGraph().edgeSet();
        Iterator<Vertex> vertexIterator = global.getGraph().getGraph().vertexSet().iterator();
        int count = 0;
        while(vertexIterator.hasNext()){
            Vertex curr_vertex = vertexIterator.next();
            if(curr_vertex.isSource())
                count++;
        }
        if(count == sourceNum) {
            ConnectivityInspector<Vertex,Edge> inspector = new ConnectivityInspector<>(graph.getGraph());
            System.out.println(inspector.isConnected());
            global.visualization();
        }else{
            global.visualization();
            System.out.println("Fail to find the Steiner Tree, " + count + " sources in total.");
        }

    }
}
