package steiner.model;

import org.jgrapht.Graph;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

import static org.jgrapht.Graphs.predecessorListOf;

public class Vertex {

    public void setServer(DistributedServer server) {
        this.server = server;
    }

    public DistributedServer getServer() {
        return server;
    }

    private DistributedServer server;

    public HashMap<Integer, Vertex> getNeighborMap() {
        return neighborMap;
    }

    private HashMap<Integer,Vertex> neighborMap = new HashMap<>();

    public int getName() {
        return name;
    }

    private pressureLocal pressureLocal;

    public pressureLocal getPressureLocal(){
        return this.pressureLocal;
    }

    public void setPressureLocal(ArrayList<Double> pressure,int turn){
        this.pressureLocal.setPressureLocal(pressure,turn);
    }

    private ConcurrentHashMap<Vertex,pressureLocal> neighborsPressureLocal = new ConcurrentHashMap<>();

    public ConcurrentHashMap<Vertex,pressureLocal> getNeighborsPressureLocal() {
        return neighborsPressureLocal;
    }

    public HashMap<Vertex, Edge> getNeighborEdgeMap() {
        return neighborEdgeMap;
    }

    private HashMap<Vertex,Edge> neighborEdgeMap = new HashMap<>();

    private final int name;

    public boolean isSource() {
        return isSource;
    }

    private boolean isSource;
    private boolean isCompleted = false;

    public Vertex getNeighborByName(int neighborName){
        return this.neighborMap.get(neighborName);
    }

    public List<Vertex> getNeighbor(Graph<Vertex,Edge> graph){
        List neighborList = predecessorListOf(graph,this);
        return neighborList;
    }

    public Set<Vertex> getNeighbors(SteinerGraph steinerGraph){
        Set<Vertex> neighbors = new HashSet<>();
        Set<Vertex> vertexSet = steinerGraph.getGraph().vertexSet();
        this.neighborMap.clear();
        for(Vertex vertex:vertexSet){
            Edge edge = steinerGraph.getGraph().getEdge(this,vertex);
            if(edge!=null) {
                Edge local = (Edge)edge.clone();
                neighborEdgeMap.put(vertex,local);
                neighbors.add(vertex);
                this.neighborMap.put(vertex.getName(),vertex);
            }
        }
        return neighbors;
    }

    public void pullNeighborPressure(byte iterationTime) throws IOException {
        neighborsPressureLocal.clear();
        this.server.askNeighbors(iterationTime);
    }

    public Vertex(int name, boolean isSource) throws IOException {
        this.name = name;
        this.isSource = isSource;
        pressureLocal = new pressureLocal(0);
        this.setServer(new DistributedServer(this));
    }

    public Vertex(int name){
        this.name = name;
        this.isSource = false;
        pressureLocal = new pressureLocal(0);
    }

    public String toString(){   //when using for visualization, the vertex's index begins from 1
        if(this.isSource())
            return "SRC " + (name+1);
        else
            return "" + (name+1);
    }

    public int hashCode(){
        return toString().hashCode();
    }

    public boolean equals(Object o){
        return (o instanceof Vertex) && (toString().equals(o.toString()));
    }

    public boolean isCompleted() {
        return isCompleted;
    }

    public void setCompleted(boolean completed) {
        isCompleted = completed;
    }
}
