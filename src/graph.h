#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <exception>
#include <map>

#ifndef _GRAPH_H_
#define _GRAPH_H_ 

namespace graph
{

class GraphException: public std::exception
{
  private:
    std::string messg;
  public: 
    GraphException(std::string msg): messg(msg){}
    const char* what() const throw(){
      const char* c = messg.c_str();
      return c;
    } 
};

class GraphAL
{
  public:
   GraphAL(bool dcted=false):directed(dcted){}
   friend std::istream& operator>>(std::istream&, GraphAL&);
   friend std::ostream& operator<<(std::ostream&, const GraphAL&);
   int vertices() const {return nVertices;}
   std::vector<int> neighbours(int) const;
  protected:
   class Node{
    public: 
     int vertex;
     Node* next;
     Node(){
      vertex=-1; next=NULL;
     }
     Node(int v, Node* nxt=NULL):vertex{v}, next{nxt}{}
     Node(int v, Node* nxt, float wt):vertex{v}, next{nxt}, weight{wt}{}
     float weight;
   };
   std::vector<Node*> adjlist;
   bool directed;
   int nEdges;
   int nVertices;
   std::vector<int> degree;
   void addNbr(int, int);
};

class GraphWAL: public GraphAL
{
  public:
   GraphWAL(){}
   GraphWAL(bool dcted=false){directed=dcted;}
   friend std::istream& operator>>(std::istream&, GraphWAL&);
   friend std::ostream& operator<<(std::ostream&, const GraphWAL&);
   void addNbr(int, int, float);
   std::map<int, float> edgeWts(int) const;
};



}  // namespace graph 
#endif // _GRAPH_H_

