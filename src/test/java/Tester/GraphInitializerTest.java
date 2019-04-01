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
    public static final int nodeNum= 100;
    public static final int sourceNum = 5;
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
        File file = new File("/home/longinus/Documents/SteinerTree/streetest/test111.txt");     //file of the network
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
            cost = sc.nextInt()/10000.0;
            adjmatrix[col][row]  = adjmatrix[row][col] = cost;
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

        List<Double> ans = new ArrayList<>();

        for(int i=1;i<2;i++) {
            for(int j=1;j<2;j++) {
                SteinerGraph graph = new SteinerGraph(adjmatrix, sources);
                Steiner_Global global = new Steiner_Global(graph, i*0.0002, 0.00001);

                global.initial_changed();
                ans.add(global.iteration(180280) + i*10000 + j*1000);
                Iterator<Vertex> vertexIterator = global.getGraph().getGraph().vertexSet().iterator();
                int count = 0;
                while (vertexIterator.hasNext()) {
                    Vertex curr_vertex = vertexIterator.next();
                    if (curr_vertex.isSource())
                        count++;
                }
                if (count == sourceNum) {
                    ConnectivityInspector<Vertex, Edge> inspector = new ConnectivityInspector<>(graph.getGraph());
                    System.out.println(inspector.isConnected());
                    global.visualization();
                } else {
                    System.out.println("Fail to find the Steiner Tree, " + count + " sources in total.");
                }
            }
        }

        Collections.sort(ans, new Comparator<Double>() {
            @Override
            public int compare(Double o1, Double o2) {
                if(o1%1000 - o2%1000 < -(10E-5))
                    return -1;
                else if(o1%1000 - o2%1000 > 10E-5)
                    return 1;
                else
                    return 0;
            }
        });

        for(double sum:ans){
            System.out.println(sum);
        }
    }
}
