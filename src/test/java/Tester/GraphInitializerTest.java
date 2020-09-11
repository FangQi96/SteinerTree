package Tester;
import steiner.algo.Steiner_Global;
import steiner.model.SteinerGraph;

import java.io.*;
import java.util.*;

public class GraphInitializerTest {
    public static final int nodeNum= 1000;
    public static final int sourceNum = 20;
    public static void randomGenerateSource(int nodeNum,int sourceNum){
        File sources = new File("/home/longinus/Documents/streetest/p2psrc.txt");
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
            for(int i=0;i<sourceNum;i++){
                int sourceIndex = arrayList.get(i);
                bufferedWriter.write(String.valueOf(sourceIndex));
                if(i!=sourceNum-1)
                    bufferedWriter.newLine();
            }
            bufferedWriter.flush();
            bufferedWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

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
        double[][] adjmatrix = new double[nodeNum][nodeNum];
        List<Integer> sources = new ArrayList<>();
        randomGenerateSource(nodeNum,sourceNum);     //randomly generate sources
        readFile(adjmatrix,sources,"1000nodes.txt","p2psrc.txt",sourceNum);

        //SteinerGraph accurateGraph = new SteinerGraph(adjmatrix, sources);
        //Steiner_Global accurateGlobal = new Steiner_Global(accurateGraph, 0.00004,0.0018);
        //accurateGlobal.initial();
        //accurateGlobal.iteration_ksub_changed(1200);
        //accurateGlobal.visualization();

        SteinerGraph approxGraph = new SteinerGraph(adjmatrix, sources);
        Steiner_Global approxGlobal = new Steiner_Global(approxGraph, 0.00004, 0.0007);
        approxGlobal.initial_approx();
        approxGlobal.iteration_ksub_changed(1200);
        //approxGlobal.visualization();
   }
}
