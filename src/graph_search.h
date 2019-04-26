#include "graph.h"
#include "model.h"
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <functional>

#ifndef GRAPH_SEARCH_
#define GRAPH_SEARCH_

namespace graph
{
 
 class PQElem
 {
   public:
    PQElem(int vert=0, float rank=0):vertex(vert), cost_rank(rank){}
    int vert() const {return vertex;}
    float key() const { return cost_rank;}
    int setKey(int rank){cost_rank = rank;}  
    friend bool operator<(const PQElem&, const PQElem&);
   private: 
    float cost_rank;
    int vertex;
 };

 bool operator<(const PQElem& pqelem1, const PQElem& pqelem2)
 {
   if (pqelem1.key() > pqelem2.key())
    return true;
   return false;
 }


 class PQueue
 {
  public:
   PQueue(std::vector<PQElem> elem_list = std::vector<PQElem>()):pqueue(elem_list){std::make_heap(pqueue.begin(), pqueue.end());}
   void push(PQElem pelem);
   PQElem extractMin();
   bool empty() const {return pqueue.empty();}
   void remove(int vertex);
  private:
   std::vector<PQElem> pqueue;
 };
 
 void PQueue::push(PQElem pelem)
 {
   // pqueue is already a heap. add one more element to end of container, and then call push heap. This has the effect of including the element before   end into the heap
   pqueue.push_back(pelem);
   std::push_heap(pqueue.begin(), pqueue.end());
 }

 PQElem PQueue::extractMin()
 {
   std::pop_heap(pqueue.begin(), pqueue.end()); // remove the element at the front and puts its at the back. 
   PQElem minElem = pqueue.back();
   pqueue.pop_back();
   return minElem; 
 }
 
 // remove element, by vertex
 void PQueue::remove(int vertex) 
 {
   if (empty())  // if empty queue, nothing to remove
     return;
   PQElem pq_elem = extractMin();
   pqueue.push_back(pq_elem);   
   
   // extracts first element, put it at the back, iterate over pqueue to find vertex, make its key 1 less than min, heapify, extractMin 
   for(auto &node: pqueue)
   {
     if (node.vert() == vertex)
     {
        node.setKey(pq_elem.key()- 1);
        std::make_heap(pqueue.begin(), pqueue.end());
        pq_elem = extractMin(); 
        break;        
     }
   }
 }

 std::vector<int> astarSearch(Model io2d_m, Modelfunc p_heuristic, GraphWAL& g, int src, int destn)
 {
  // data structures used in a* search
  
  //std::cerr<<"entered astar \n"<<"src: :"<<src<<" destn: "<<destn<<"\n";
  graph::PQueue pqOpen; 
  std::unordered_set<int> closed, open;
  std::vector<float> gCosts(g.vertices()+1, 0); // 1-indexed
  std::vector<int> parent(g.vertices()+1, 0); 
  // initialize  
  pqOpen.push(graph::PQElem(src, 0)); 
  open.insert(src);
  graph::PQElem currNode; 
  while((currNode = pqOpen.extractMin()).vert()!= destn)
  {
   int curr_vertex = currNode.vert();
   if (open.find(curr_vertex) != open.end())
   {
    open.erase(open.find(curr_vertex));
    pqOpen.remove(curr_vertex);
   }
   closed.insert(curr_vertex);
   std::map<int, float> currWtsDict = g.edgeWts(curr_vertex);
   for (auto nbr: g.neighbours(curr_vertex))
   {
     //float cost = gCosts[curr_vertex] + 1 ;
     float cost = gCosts[curr_vertex] + currWtsDict[nbr];
     if (open.find(nbr) != open.end() && cost < gCosts[nbr])
     {
      open.erase(open.find(nbr));
      pqOpen.remove(nbr);
     }
     if (closed.find(nbr) != closed.end() && cost < gCosts[nbr])
      closed.erase(closed.find(nbr));
     if (open.find(nbr) == open.end() && closed.find(nbr) == closed.end())
     {
       parent[nbr] =curr_vertex; 
       gCosts[nbr] = cost;
       pqOpen.push(graph::PQElem(nbr, gCosts[nbr] + std::invoke(p_heuristic, io2d_m, nbr, destn))); 
       open.insert(nbr); 
     }
   }
  }
  std::vector<int> path;
  int elem = destn;
  do{
   path.push_back(elem);
  }while((elem = parent[elem]) != 0);

  return path;
  
 }  

} // namespace graph




#endif // GRAPH_SEARCH_
