#pragma once

#include <vector>
#include <unordered_map>
#include <map>
#include <unordered_set>
#include <string>
#include <cstddef>
#include "graph.h"

class Model
{
public:
    struct Node {
        double x = 0.f;
        double y = 0.f;
    };
    
    struct Way {
        std::vector<int> nodes;
    };
    
    struct Road {
        enum Type { Invalid, Unclassified, Service, Residential,
            Tertiary, Secondary, Primary, Trunk, Motorway, Footway };
        int way;
        Type type;
    };
    
    struct Railway {
        int way;
    };    
    
    struct Multipolygon {
        std::vector<int> outer;
        std::vector<int> inner;
    };
    
    struct Building : Multipolygon {};
    
    struct Leisure : Multipolygon {};
    
    struct Water : Multipolygon {};
    
    struct Landuse : Multipolygon {
        enum Type { Invalid, Commercial, Construction, Grass, Forest, Industrial, Railway, Residential };
        Type type;
    };
    
    //rk: adding new model object - route
    struct Route {
       int way;
    };
     
    struct Location
    {
      int way;
    };      
     
    Model( const std::vector<std::byte> &xml );
    
    auto MetricScale() const noexcept { return m_MetricScale; }    
    
    auto &Nodes() const noexcept { return m_Nodes; }
    auto &Ways() const noexcept { return m_Ways; }
    auto &Roads() const noexcept { return m_Roads; }
    auto &Buildings() const noexcept { return m_Buildings; }
    auto &Leisures() const noexcept { return m_Leisures; }
    auto &Waters() const noexcept { return m_Waters; }
    auto &Landuses() const noexcept { return m_Landuses; }
    auto &Railways() const noexcept { return m_Railways; }
    auto &Routes() const noexcept {return m_Routes;}
    auto &Locations() const noexcept {return m_Locations;}
    // rk
    void generateGraph(graph::GraphWAL & g);
    float eucledianDistance(int, int);
    void addSrcDestn(std::string src_string, std::string destn_string);
    int Src(){return nodeToVertex[src_node];}
    int Destn(){return nodeToVertex[destn_node];}    
    void addRoute(std::vector<int> gVertices); 
private:
    void AdjustCoordinates();
    void BuildRings( Multipolygon &mp );
    void LoadData(const std::vector<std::byte> &xml);
    
    std::vector<Node> m_Nodes;
    std::vector<Way> m_Ways;
    std::vector<Road> m_Roads;
    std::vector<Railway> m_Railways;
    std::vector<Building> m_Buildings;
    std::vector<Leisure> m_Leisures;
    std::vector<Water> m_Waters;
    std::vector<Landuse> m_Landuses;
    std::vector<Route> m_Routes;
    std::vector<Location> m_Locations; 
        
    double m_MinLat = 0.;
    double m_MaxLat = 0.;
    double m_MinLon = 0.;
    double m_MaxLon = 0.;
    double m_MetricScale = 1.f;

    //rk: vertex to node mapping
    std::map<int,int> vertexToNode;
    std::map<int, int> nodeToVertex; 
    std::map<int, std::map<int, float>> adjList; 
    int src_node, destn_node;
    void addSrc(std::string src_string);
    void addDestn(std::string destn_string);
    double lat2ym(double lat);
    double lon2xm(double lon);
    int findNearest(struct Node nd);
};


typedef float (Model::*Modelfunc)(int, int); // typedef of pointer to Model member function
