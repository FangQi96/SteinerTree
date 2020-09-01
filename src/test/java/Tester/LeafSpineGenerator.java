package Tester;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import static org.apache.commons.lang3.RandomUtils.nextInt;

public class LeafSpineGenerator {
    public static void main(String args[]){
        int spineNum = 10;
        int leafNum = 30;
        int serverEachLeaf = 2;
        int terminalNum = 15;
        ArrayList<Integer> spines = new ArrayList<>();
        for(int i=1;i<=spineNum;i++)
            spines.add(i);
        ArrayList<Integer> leaves = new ArrayList<>();
        for(int i=spineNum+1;i<=spineNum+leafNum;i++)
            leaves.add(i);
        ArrayList<Integer> servers = new ArrayList<>();
        for(int i=spineNum+leafNum+1;i<=spineNum+leafNum+serverEachLeaf*leafNum;i++)
            servers.add(i);

        File graph = new File("/Users/longinus/Documents/streetest/test111.txt");
        try {
            graph.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(graph));
            for(Integer i:spines){
                for (Integer j:leaves){
                    bufferedWriter.write(i+" "+j+" "+ nextInt(100,1000));
                    bufferedWriter.newLine();
                }
            }
            for (int i = 0, j = 0; i < leaves.size() && j < servers.size(); i++, j += serverEachLeaf) {
                for(int k=0;k<serverEachLeaf;k++) {
                    bufferedWriter.write(leaves.get(i) + " " + servers.get(j+k) + " " + nextInt(100, 1000));
                    if (j != servers.size() - 1)
                        bufferedWriter.newLine();
                }
            }
            bufferedWriter.flush();
            bufferedWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        File terminal = new File("/Users/longinus/Documents/streetest/p2psrc.txt");
        try {
            terminal.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            Collections.shuffle(leaves);
            BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(terminal));
            for(int i=0;i<terminalNum;i++){
                bufferedWriter.write(String.valueOf(leaves.get(i)));
                if(i!=terminalNum-1)
                    bufferedWriter.newLine();
            }
            bufferedWriter.flush();
            bufferedWriter.close();
        } catch (IOException e){
            e.printStackTrace();
        }
    }
}
