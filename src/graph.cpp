#include "graph.h"

namespace graph
{

std::vector<int> GraphAL::neighbours(int vertex) const
{
 std::vector<int> nbrs;
 Node* nd = adjlist[vertex];
 while(nd != NULL)
 {
  nbrs.push_back(nd->vertex);
  nd = nd->next;
 }
 return nbrs; 
}
void GraphAL::addNbr(int src, int nbr)
{
  Node* head = adjlist[src]; 
  Node* nd = new Node(nbr);
  adjlist[src] = nd;
  nd->next = head;
  degree[src]++; 
}


std::istream& operator>>(std::istream& is, GraphAL& g)
{
 //read number of vertices and edges
 std::istringstream iss0;
 std::string line0;
 std::getline(is, line0);
 iss0.str(line0);
 iss0>>g.nVertices>>g.nEdges;
 g.adjlist.resize(g.nVertices+1);
 g.degree.resize(g.nVertices+1);
 //read neighbours of each vertex 
 int vertex = 0;
 for(int i=1; i<=g.nVertices; i++)
 { 
  // read one line and process it
  std::string line;
  std::getline(is, line);
  std::istringstream iss{line};
  iss>>vertex; // first vertex is the source node's index itself, discard it. 
  if (!iss.eof()) // if first vertex is the only vertex, skip the rest. 
  {
    while(1)
    {
      iss>>vertex;
      if (iss.eof())
        break;
      // insert new node at the front of the list
      g.addNbr(i, vertex); 
    }
    g.addNbr(i, vertex);//for some reason iss>>vertex sets eof on read last interger in the line 
  }
 }
 return is;  
}

std::ostream& operator<<(std::ostream& os, const GraphAL& g)
{
  os<<g.nVertices<<" "<<g.nEdges<<"\n";
  int vertex = 1;
  for(int i=1; i<=g.nVertices; i++)
  {
   os<<i<<" ";
   GraphAL::Node* nd = g.adjlist[i];
   while(nd!= NULL){
     os<<nd->vertex<<" ";
     nd = nd->next;
   }
   os<<"\n";
  }
  return os;
}


std::map<int,float> GraphWAL::edgeWts(int vertex) const
{
 std::map<int, float> edgeWeights;
 Node* nd = adjlist[vertex];
 while(nd != NULL)
 {
  edgeWeights[nd->vertex] = nd->weight;
  nd = nd->next;
 }
 return edgeWeights; 
}

void GraphWAL::addNbr(int src, int nbr, float wt)
{
  Node* head = adjlist[src]; 
  Node* nd = new Node(nbr, NULL, wt);
  adjlist[src] = nd;
  nd->next = head;
  degree[src]++; 
}

std::istream& operator>>(std::istream& is, GraphWAL& g)
{
 //read number of vertices and edges
 std::istringstream iss0;
 std::string line0;
 std::getline(is, line0);
 iss0.str(line0);
 iss0>>g.nVertices>>g.nEdges;
 g.adjlist.resize(g.nVertices+1);
 g.degree.resize(g.nVertices+1);
 //read neighbours of each vertex 
 int vertex = 0; 
 float weight=1.0;
 for(int i=1; i<=g.nVertices; i++)
 { 
  // read one line and process it
  std::string line;
  std::getline(is, line);
  std::istringstream iss{line};
  iss>>vertex; // first vertex is the source node's index itself, discard it. 
  if (!iss.eof()) // if first vertex is the only vertex, skip the rest. 
  {
    while(1)
    {
      iss>>vertex>>weight;  // read in pairs, destn vertex and edge (src-destn) weight
      if (iss.eof())
        break;
      // insert new node at the front of the list
      g.addNbr(i, vertex, weight); 
    }
    g.addNbr(i, vertex, weight);//for some reason iss>>vertex sets eof on read last interger in the line 
  }
 }
 return is;  
}

std::ostream& operator<<(std::ostream& os, const GraphWAL& g)
{
  os<<g.nVertices<<" "<<g.nEdges<<"\n";
  int vertex = 1;
  for(int i=1; i<=g.nVertices; i++)
  {
   os<<i<<" ";
   GraphAL::Node* nd = g.adjlist[i];
   while(nd!= NULL){
     os<<nd->vertex<<" "<<nd->weight<<" ";
     nd = nd->next;
   }
   os<<"\n";
  }
  return os;
}

}  // namespace graph 

