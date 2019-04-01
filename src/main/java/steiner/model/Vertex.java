package steiner.model;

import org.jgrapht.Graph;

import java.awt.geom.Point2D;
import java.util.List;

import static org.jgrapht.Graphs.predecessorListOf;

public class Vertex {
    public int getName() {
        return name;
    }

    private final int name;

    private Point2D coordinate = new Point2D.Double();      //For potential need of calculating the physical distance

    public boolean isSource() {
        return isSource;
    }

    public void setSource(boolean source) {
        isSource = source;
    }

    private boolean isSource;


    public List<Vertex> getNeighbor(Graph graph){
        List neighbors = predecessorListOf(graph,this);
        return neighbors;
    }

    public Vertex(int name, boolean isSource){
        this.name = name;
        this.isSource = isSource;
    }

    public Vertex(int name){
        this.name = name;
        this.isSource = false;
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
}
