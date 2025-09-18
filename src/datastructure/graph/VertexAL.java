package datastructure.graph;

import java.util.LinkedList;

/**
 * Implementation of the vertexes containing <code>data</code> objects of a generic 
 * Graph data structure composed of edges of type EdgeAL and implemented using
 * Adjacent Lists. 
 * Besides the data object, the vertexes contain the pointer to the list of adjacent vertexes.
 * @param <D> type of the data object 
 */	
public class VertexAL<D> implements Vertex<D> {

	/** the data element in the vertex */	
	protected D data;
	
	/** the adjacent list of the vertex */	
	protected LinkedList<EdgeAL<D>> adjac;

	/**
   	 * Constructs a vertex for a graph implemented using adjacent lists
	 * @param data data to insert in the vertex
	 */			
	public VertexAL(D data) { 
		this.data = data;
		adjac =  new LinkedList<EdgeAL<D>>();
	}
	
	/**
	 * Returns the vertex data
	 * @return the vertex data
	 */
	public D getData() {
		return this.data;
	}

}
