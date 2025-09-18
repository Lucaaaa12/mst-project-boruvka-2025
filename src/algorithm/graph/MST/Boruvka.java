package algorithm.graph.MST;

import datastructure.graph.*;
import datastructure.unionfind.*;
import java.util.*;

/**
 * Generic graph computation of the Minimum Spanning Tree using the Boruvka algorithm
 * @param <D> type of the data object in the graph vertexes
 */
public class Boruvka<D> implements MST<D> {

	/**
	 * Computes a Minimum Spanning Tree of a passed graph using the Boruvka algorithm 
	 * and returns the computed spanning tree represented as a graph
	 * @param graph the graph for which the Minimum Spanning Tree must be computed
	 * @return the graph representing the computed Minimum Spanning Tree
	 */	
    @Override
	public Graph<D> MinimumSpanningTree(Graph<D> graph) {

        QuickFindSize<Vertex<D>> unionFind = new QuickFindSize<Vertex<D>>();
        
        // Mappa per convertire i vertici del grafo originale in quelli del grafo MST
        HashMap<Vertex<D>, Vertex<D>> vertexMap = new HashMap<>(graph.vertexNum());
        
        // Mappa per convertire i vertici del grafo al relativo nodo Union-Find
        HashMap<Vertex<D>, UnionFindNode<Vertex<D>>> nodeMap = new HashMap<>(graph.vertexNum());
        
        GraphAL<D> mst = new GraphAL<D>(); // Grafo MST da restituire in output
        
        // Per ogni vertice, aggiungo un nodo nel MST e creo un set disgiunto Union-Find.
		// Uso due mappe per collegare i vertici originali con i nodi Union-Find e i corrispettivi vertici MST.
        for (Vertex<D> v : graph.vertexes()) {
            Vertex<D> mstVertex = mst.addVertex(v.getData());
            UnionFindNode<Vertex<D>> ufNode = unionFind.makeSet(v);
            nodeMap.put(v, ufNode);
            vertexMap.put(v, mstVertex);
        }

        // Numero iniziale di componenti connesse (una per ogni vertice)
        int numComponents = graph.vertexNum();

        // Ciclo while termina quando si ottiene una singola componente, ovvero quando l'MST è completo
        while (numComponents >= 2) {

            // Mappa temporanea che associa a ogni componente  
			// l'arco di peso minimo uscente che collega quella componente a una diversa.
            HashMap<UnionFindNode<Vertex<D>>, Edge<D>> minEdgeMap = new HashMap<>(numComponents);

            // Per ogni arco, se collega due componenti diverse, verifico se se è il minimo uscente dalla componente sorgente
			// Ignoro archi che collegano vertici della stessa componente per evitare cicli.
            for (Edge<D> edge : graph.edges()) {

                Vertex<D> u = edge.getSource();
                Vertex<D> v = edge.getDest();

                // Trovo i rappresentanti dei due vertici nelle rispettive componenti Union-Find
                UnionFindNode<Vertex<D>> sourceNode = unionFind.find(nodeMap.get(u));                
                UnionFindNode<Vertex<D>> destNode = unionFind.find(nodeMap.get(v));

                if (destNode.equals(sourceNode)) continue;

                    // Seleziono l’arco di peso minimo uscente dalla componente di u
                    Edge<D> currMinEdgeU = minEdgeMap.get(sourceNode);
                    if (currMinEdgeU == null || currMinEdgeU.getWeight() > edge.getWeight()) {
                        minEdgeMap.put(sourceNode, edge); 
                    }
                
            }

			// Per ogni arco minimo selezionato:
			// Verifico che colleghi componenti distinte, quindi aggiungo l’arco al MST
			// e unisco le componenti corrispondenti nel Union-Find.
            for (Edge<D> edge : minEdgeMap.values()) {

                // Vertici originali dal grafo di input
                Vertex<D> originalSource = edge.getSource();
                Vertex<D> originalDest = edge.getDest();
                
                // Trovo i rappresentanti dei due vertici nelle rispettive componenti Union-Find
                UnionFindNode<Vertex<D>> sourceNode = unionFind.find(nodeMap.get(originalSource));
                UnionFindNode<Vertex<D>> destNode = unionFind.find(nodeMap.get(originalDest));
                
                // Vertici equivalenti nel grafo MST
                Vertex<D> mstSource = vertexMap.get(originalSource);              
                Vertex<D> mstDest = vertexMap.get(originalDest);

                // Se appartengono già alla stessa componente, salto l'arco 
                if (sourceNode.equals(destNode)) continue;
                
                mst.addEdge(mstSource, mstDest, edge.getWeight());
                mst.addEdge(mstDest, mstSource, edge.getWeight());

                unionFind.union(destNode, sourceNode);
				// Decremento il numero di componenti connesse
                numComponents--;  
            }
        }

        return mst;

	}
	
}
