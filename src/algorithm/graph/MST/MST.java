package algorithm.graph.MST;

import datastructure.graph.*;

/**
 * Generic graph computation of the Minimum Spanning Tree
 * @param <D> type of the data object in the graph vertexes
 */
public interface MST<D> {

	/**
	 * Computes a Minimum Spanning Tree of a passed graph and returns
	 * the computed spanning tree represented as a graph.
	 * @param graph the graph for which the Minimum Spanning Tree must be computed
	 * @return the graph representing the computed Minimum Spanning Tree
	 */	
	public Graph<D> MinimumSpanningTree(Graph<D> graph);
	
}
