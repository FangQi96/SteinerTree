package steiner.algo;

import steiner.model.*;
import java.util.ArrayList;
import java.util.Set;

public class Steiner_Distributed {
    private Steiner_Global steiner_global;
    private double I_0 = 100;
    private double delta = 1.4;
    private double rho = 0.95;

    public Steiner_Global getSteiner_global() {
        return steiner_global;
    }

    public Steiner_Distributed(SteinerGraph graph, double delta, double rho, double edge_threshold){
        steiner_global = new Steiner_Global(graph,delta,rho,edge_threshold);
        this.delta = delta;
        this.rho = rho;
    }

    public void globalPreCalculation(int T_c){
    	/********************************************************
    	*After calling this, send down the graph to the vertices*
     	*********************************************************/
        steiner_global.initial();
        steiner_global.iteration_ksub_changed(T_c);
    }
/*
    public void collectResult(){
        for(Edge edge:steiner_global.getGraph().getGraph().edgeSet()){
            Vertex target = (Vertex)edge.getTarget();
            Vertex source = (Vertex)edge.getSource();
            double conductSource = source.getNeighborEdgeMap().get(target).getConductivity();
            double conductTarget = target.getNeighborEdgeMap().get(source).getConductivity();
            edge.setConductivity((conductSource+conductTarget)/2);
        }
    }

    public void visualization(){
        collectResult();
        steiner_global.cutEdgeProportional(0.1);     //To make the final graph visualization more human-readable

        System.out.println("Final Edge Sum: " + steiner_global.getGraph().getEdgeWeightSum());
        steiner_global.visualization();
    }

    public void vertexInitial() throws Exception {
        /**********************************************************
    	*Convert the global pressure based on sink into local pre-*
        *ssure based on vertex, send them down and then update the*
        *neighbor's pressure via Socket communication**************
     	***********************************************************/
/*
        Set<Vertex> sourceSet = steiner_global.getSourceSet();
        for(Vertex vertex:steiner_global.getGraph().getGraph().vertexSet()){
            ArrayList<Double> vertexPressure = new ArrayList<>();
            for(Vertex sink:sourceSet){
                int sinkIndex = steiner_global.getGraph().getSourceList().indexOf(sink.getName());
                double[] pressureWithCurrentSink = steiner_global.getGraph().getPressure().get(sinkIndex);
                vertexPressure.add(pressureWithCurrentSink[vertex.getName()]);
            }
            vertex.setPressureLocal(vertexPressure,0);
            vertexPressure.toArray(vertex.getServer().getSlideWindow()[0]);
        }

        Set<Vertex> set = steiner_global.getGraph().getGraph().vertexSet();

        for(Vertex vertex:set){
            vertex.getServer().setVertexNumber(steiner_global.getGraph().getAdjmatrix().length);
            vertex.getServer().setSourceSet(steiner_global.getSourceSet());
            vertex.getNeighbors(steiner_global.getGraph());
        }
        //DistributedServer.setInitialFlag(true);     //Vertices are ready now
        //Thread.sleep(1000); //This is used for synchronized pressure updating
    }

/*    public void distributedIteration(int inner,int outer) throws Exception {
        double[][] adjmatrix = steiner_global.getGraph().getAdjmatrix();
        for(int i=0;i<outer;i++) {
            for (Vertex vertex : steiner_global.getGraph().getGraph().vertexSet()) {
                for (int j = 0; j < inner; j++) {
                    int count = 0;
                    for (Vertex sink : steiner_global.getSourceSet()) {
                        double sum_above = 0;
                        double sum_below = 0.000001;
                        for (ConcurrentHashMap.Entry<Vertex, pressureLocal> entry : vertex.getNeighborsPressureLocal().entrySet()) {
                            Edge edge = steiner_global.getGraph().getGraph().getEdge(vertex, entry.getKey());
                            double flux = Math.abs(edge.getConductivity() * (vertex.getPressureLocal().getPressure().get(count) - entry.getValue().getPressure().get(count)) / edge.getWeight());
                            double new_conductivity = (Math.pow(flux, delta) / (I_0 * I_0 / adjmatrix.length * adjmatrix.length + Math.pow(flux, delta))) + rho * edge.getConductivity();
                            edge.setConductivity(new_conductivity);
                            /********************************************************
                            *Don't cut, leave it to the graph(AKA global controller)*
                            *********************************************************/
/*                            sum_below = sum_below + new_conductivity / edge.getWeight();
                            sum_above = sum_above + new_conductivity * entry.getValue().getPressure().get(count) / edge.getWeight();

                        }

                        if (vertex.isSource()) {
                            if (vertex.getName() == sink.getName())
                                sum_above = sum_above - (steiner_global.getSourceSet().size() - 1) * I_0;
                            else
                                sum_above = sum_above + I_0;
                        } else
                            ;
                        vertex.getPressureLocal().getPressure().set(count,sum_above/sum_below);
                        count++;
                    }
                }

            }
            for(Vertex vertex:steiner_global.getGraph().getGraph().vertexSet()){
                vertex.pullNeighborPressure();
            }
            Thread.sleep(100);     //This is used for synchronized pressure updating
        }
        //for(Vertex vertex:steiner_global.getGraph().getGraph().vertexSet())     //Stop all receiving threads
            //vertex.getServerThread().interrupt();
    }*/
}
