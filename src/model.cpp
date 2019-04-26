#include "model.h"
#include "external/pugixml/src/pugixml.hpp"
#include <iostream>
#include <string_view>
#include <cmath>
#include <algorithm>
#include <assert.h>
#include <unordered_set>
#include <fstream>
#include <istream>
#include <ostream>


static Model::Road::Type String2RoadType(std::string_view type)
{
    if( type == "motorway" )        return Model::Road::Motorway;
    if( type == "trunk" )           return Model::Road::Trunk;
    if( type == "primary" )         return Model::Road::Primary;
    if( type == "secondary" )       return Model::Road::Secondary;    
    if( type == "tertiary" )        return Model::Road::Tertiary;
    if( type == "residential" )     return Model::Road::Residential;
    if( type == "living_street" )   return Model::Road::Residential;    
    if( type == "service" )         return Model::Road::Service;
    if( type == "unclassified" )    return Model::Road::Unclassified;
    if( type == "footway" )         return Model::Road::Footway;
    if( type == "bridleway" )       return Model::Road::Footway;
    if( type == "steps" )           return Model::Road::Footway;
    if( type == "path" )            return Model::Road::Footway;
    if( type == "pedestrian" )      return Model::Road::Footway;    
    return Model::Road::Invalid;    
}

static Model::Landuse::Type String2LanduseType(std::string_view type)
{
    if( type == "commercial" )      return Model::Landuse::Commercial;
    if( type == "construction" )    return Model::Landuse::Construction;
    if( type == "grass" )           return Model::Landuse::Grass;
    if( type == "forest" )          return Model::Landuse::Forest;
    if( type == "industrial" )      return Model::Landuse::Industrial;
    if( type == "railway" )         return Model::Landuse::Railway;
    if( type == "residential" )     return Model::Landuse::Residential;    
    return Model::Landuse::Invalid;
}

Model::Model( const std::vector<std::byte> &xml )
{
    LoadData(xml);
    //m_Route.push_back(Route{0}) 
    AdjustCoordinates();
    std::sort(m_Roads.begin(), m_Roads.end(), [](const auto &_1st, const auto &_2nd){
        return (int)_1st.type < (int)_2nd.type; 
    });
}

void Model::LoadData(const std::vector<std::byte> &xml)
{
    using namespace pugi;
    
    xml_document doc;
    if( !doc.load_buffer(xml.data(), xml.size()) )
        throw std::logic_error("failed to parse the xml file");
    
    if( auto bounds = doc.select_nodes("/osm/bounds"); !bounds.empty() ) {
        auto node = bounds.first().node();
        m_MinLat = atof(node.attribute("minlat").as_string());
        m_MaxLat = atof(node.attribute("maxlat").as_string());
        m_MinLon = atof(node.attribute("minlon").as_string());
        m_MaxLon = atof(node.attribute("maxlon").as_string());
    }
    else 
        throw std::logic_error("map's bounds are not defined");

    std::unordered_map<std::string, int> node_id_to_num;
    for( const auto &node: doc.select_nodes("/osm/node") ) {
        node_id_to_num[node.node().attribute("id").as_string()] = (int)m_Nodes.size();
        m_Nodes.emplace_back();        
        m_Nodes.back().y = atof(node.node().attribute("lat").as_string());
        m_Nodes.back().x = atof(node.node().attribute("lon").as_string());
    }

    std::unordered_map<std::string, int> way_id_to_num;    
    for( const auto &way: doc.select_nodes("/osm/way") ) {
        auto node = way.node();
        
        const auto way_num = (int)m_Ways.size();
        way_id_to_num[node.attribute("id").as_string()] = way_num;
        m_Ways.emplace_back();
        auto &new_way = m_Ways.back();
        
        for( auto child: node.children() ) {
            auto name = std::string_view{child.name()}; 
            if( name == "nd" ) {
                auto ref = child.attribute("ref").as_string();
                if( auto it = node_id_to_num.find(ref); it != end(node_id_to_num) )
                    new_way.nodes.emplace_back(it->second);
            }
            else if( name == "tag" ) {
                auto category = std::string_view{child.attribute("k").as_string()};
                auto type = std::string_view{child.attribute("v").as_string()};
                if( category == "highway" ) {
                    if( auto road_type = String2RoadType(type); road_type != Road::Invalid ) {
                        m_Roads.emplace_back();
                        m_Roads.back().way = way_num;
                        m_Roads.back().type = road_type;
                    }
                }
                if( category == "railway" ) {
                    m_Railways.emplace_back();
                    m_Railways.back().way = way_num;
                }                
                else if( category == "building" ) {
                    m_Buildings.emplace_back();
                    m_Buildings.back().outer = {way_num};
                }
                else if( category == "leisure" ||
                        (category == "natural" && (type == "wood"  || type == "tree_row" || type == "scrub" || type == "grassland")) ||
                        (category == "landcover" && type == "grass" ) ) {
                    m_Leisures.emplace_back();
                    m_Leisures.back().outer = {way_num};
                }
                else if( category == "natural" && type == "water" ) {
                    m_Waters.emplace_back();
                    m_Waters.back().outer = {way_num};
                }
                else if( category == "landuse" ) {
                    if( auto landuse_type = String2LanduseType(type); landuse_type != Landuse::Invalid ) {
                        m_Landuses.emplace_back();
                        m_Landuses.back().outer = {way_num};
                        m_Landuses.back().type = landuse_type;
                    }                    
                }
            }
        }
    }
    
    for( const auto &relation: doc.select_nodes("/osm/relation") ) {
        auto node = relation.node();
        auto noode_id = std::string_view{node.attribute("id").as_string()};
        std::vector<int> outer, inner;
        auto commit = [&](Multipolygon &mp) {
            mp.outer = std::move(outer);
            mp.inner = std::move(inner);
        };
        for( auto child: node.children() ) {
            auto name = std::string_view{child.name()}; 
            if( name == "member" ) {
                if( std::string_view{child.attribute("type").as_string()} == "way" ) {
                    if( !way_id_to_num.count(child.attribute("ref").as_string()) )
                        continue;
                    auto way_num = way_id_to_num[child.attribute("ref").as_string()];
                    if( std::string_view{child.attribute("role").as_string()} == "outer" )
                        outer.emplace_back(way_num);
                    else
                        inner.emplace_back(way_num);
                }
            }
            else if( name == "tag" ) { 
                auto category = std::string_view{child.attribute("k").as_string()};
                auto type = std::string_view{child.attribute("v").as_string()};
                if( category == "building" ) {
                    commit( m_Buildings.emplace_back() );
                    break;
                }
                if( category == "natural" && type == "water" ) {
                    commit( m_Waters.emplace_back() );
                    BuildRings(m_Waters.back());
                    break;
                }
                if( category == "landuse" ) {
                    if( auto landuse_type = String2LanduseType(type); landuse_type != Landuse::Invalid ) {
                        commit( m_Landuses.emplace_back() );
                        m_Landuses.back().type = landuse_type;
                        BuildRings(m_Landuses.back());
                    }
                    break;
                }
            }
        }
    }
}

void Model::AdjustCoordinates()
{    
    const auto pi = 3.14159265358979323846264338327950288;
    const auto deg_to_rad = 2. * pi / 360.;
    const auto earth_radius = 6378137.;
    const auto lat2ym = [&](double lat) { return log(tan(lat * deg_to_rad / 2 +  pi/4)) / 2 * earth_radius; };
    const auto lon2xm = [&](double lon) { return lon * deg_to_rad / 2 * earth_radius; };     
    const auto dx = lon2xm(m_MaxLon) - lon2xm(m_MinLon);
    const auto dy = lat2ym(m_MaxLat) - lat2ym(m_MinLat);
    const auto min_y = lat2ym(m_MinLat);
    const auto min_x = lon2xm(m_MinLon);
    m_MetricScale = std::min(dx, dy);
    for( auto &node: m_Nodes ) {
        node.x = (lon2xm(node.x) - min_x) / m_MetricScale;
        node.y = (lat2ym(node.y) - min_y) / m_MetricScale;        
    }
}

static bool TrackRec(const std::vector<int> &open_ways,
                     const Model::Way *ways,
                     std::vector<bool> &used,
                     std::vector<int> &nodes) 
{
    if( nodes.empty() ) {
        for( int i = 0; i < open_ways.size(); ++i )
            if( !used[i] ) {
                used[i] = true;
                const auto &way_nodes = ways[open_ways[i]].nodes;
                nodes = way_nodes;
                if( TrackRec(open_ways, ways, used, nodes) )
                    return true;
                nodes.clear();
                used[i] = false;
            }
        return false;
    }
    else {
        const auto head = nodes.front();
        const auto tail = nodes.back();
        if( head == tail && nodes.size() > 1 )
            return true;
        for( int i = 0; i < open_ways.size(); ++i )
            if( !used[i] ) {
                const auto &way_nodes = ways[open_ways[i]].nodes;
                const auto way_head = way_nodes.front();
                const auto way_tail = way_nodes.back();
                if( way_head == tail || way_tail == tail ) {
                    used[i] = true;
                    const auto len = nodes.size();
                    if( way_head == tail ) 
                        nodes.insert(nodes.end(), way_nodes.begin(), way_nodes.end());
                    else
                        nodes.insert(nodes.end(), way_nodes.rbegin(), way_nodes.rend());
                    if( TrackRec(open_ways, ways, used, nodes) )
                        return true;
                    nodes.resize(len);                    
                    used[i] = false;
                }
            }
        return false;
    }
}

static std::vector<int> Track(std::vector<int> &open_ways, const Model::Way *ways)
{
    assert( !open_ways.empty() );
    std::vector<bool> used(open_ways.size(), false);
    std::vector<int> nodes;    
    if( TrackRec(open_ways, ways, used, nodes) )
        for( int i = 0; i < open_ways.size(); ++i )
            if( used[i] )
                open_ways[i] = -1;
    return nodes;
}

void Model::BuildRings( Multipolygon &mp )
{
    auto is_closed = []( const Model::Way &way ) {
        return way.nodes.size() > 1 && way.nodes.front() == way.nodes.back();    
    };

    auto process = [&]( std::vector<int> &ways_nums ) {
        auto ways = m_Ways.data();
        std::vector<int> closed, open;
        
        for( auto &way_num: ways_nums )
            (is_closed(ways[way_num]) ? closed : open).emplace_back(way_num);  
        
        while( !open.empty() ) {            
            auto new_nodes = Track(open, ways);
            if( new_nodes.empty() )
                break;
            open.erase(std::remove_if(open.begin(), open.end(), [](auto v){return v < 0;}), open.end() );
            closed.emplace_back( (int)m_Ways.size() );
            Model::Way new_way;
            new_way.nodes = std::move(new_nodes);
            m_Ways.emplace_back(new_way);
        }        
        std::swap(ways_nums, closed);        
    };

    process(mp.outer);
    process(mp.inner);
}

void Model::generateGraph(graph::GraphWAL& g)
{
 std::unordered_set<int> navWays;
 for(auto road: m_Roads)
   navWays.insert(road.way); 
 int n_edges = 0;
 for(auto way: navWays)
 {
  int way_size;
  if((way_size = (m_Ways[way].nodes).size()) == 1)
   continue;
  std::vector<int> nodes(m_Ways[way].nodes);
  for(int i=0; i<(way_size-1);i++)
  {
    float distance = std::sqrt(std::pow(m_Nodes[nodes[i]].x - m_Nodes[nodes[i+1]].x, 2) + std::pow(m_Nodes[nodes[i]].y - m_Nodes[nodes[i+1]].y, 2)); 
    (adjList[nodes[i]])[nodes[i+1]] = distance; 
    (adjList[nodes[i+1]])[nodes[i]] = distance; 
    n_edges += 1;
  } 
 }
 
 int n_vertices = 0; 
 // translate vertex indices in model, to vertex indices in graph 
 for(const auto &elem: adjList)
 {
   n_vertices += 1; 
   vertexToNode[n_vertices] = elem.first;
   nodeToVertex[elem.first] = n_vertices;
 }
 
 std::string adjlist_str="";
 for(const auto &elem: adjList)
 {
   adjlist_str = adjlist_str + std::to_string(nodeToVertex[elem.first]); 
   for(const auto &elem2: elem.second) // iterate over dict of (vertex, wts)
     adjlist_str = adjlist_str + " " + std::to_string(nodeToVertex[elem2.first]) + " " + std::to_string(elem2.second); 
   adjlist_str = adjlist_str + "\n"; 
 }
 
 // print to file and then read into graph 
 adjlist_str = std::to_string(n_vertices) + " " + std::to_string(n_edges) + "\n" + adjlist_str;
 std::ofstream ofs;
 ofs.open("scratch/temp", std::ofstream::trunc);
 if (!ofs)
   std::cerr<<"failed to open file temp\n";
 ofs<<adjlist_str;
 ofs.close();
 // read into graph 
 std::ifstream ifs{"scratch/temp"};
 ifs>>g;
}

float Model::eucledianDistance(int v1, int v2)
{
  int node1 = vertexToNode[v1];
  int node2 = vertexToNode[v2];
  return std::sqrt(std::pow(m_Nodes[node1].x - m_Nodes[node2].x, 2) + std::pow(m_Nodes[node1].y - m_Nodes[node2].y, 2)); 
}


void Model::addSrcDestn(std::string src_string, std::string destn_string)
{
  if (src_string.empty() || destn_string.empty())
    return;
  addSrc(src_string);
  addDestn(destn_string);
  
}

void Model::addSrc(std::string src_string)
{
  double lat=0, lon=0; 
  std::stringstream ss;
  ss<<src_string;
  char c;
  ss>>lon;
  ss>>c;
  ss>>lat;
  double lat_ym = lat2ym(lat);
  double lon_xm = lon2xm(lon);
  src_node = findNearest(Node{lon_xm, lat_ym});
  std::cerr<<"actual src = "<<lon_xm<<" lon & "<<lat_ym<<" lat \n";
  std::cerr<<"src node = "<<m_Nodes[src_node].x<<" lon & "<<m_Nodes[src_node].y<<" lat \n";
  Way loc;
  loc.nodes.push_back(src_node);
  
  m_Ways.push_back(loc); 
  m_Locations.push_back(Location{m_Ways.size()-1}); 
}

void Model::addDestn(std::string destn_string)
{
  double lat=0, lon=0; 
  std::stringstream ss;
  ss<<destn_string;
  char c;
  ss>>lon;
  ss>>c;
  ss>>lat;
  double lat_ym = lat2ym(lat);
  double lon_xm = lon2xm(lon);
  destn_node = findNearest(Node{lon_xm,lat_ym}); 
  std::cerr<<"actual destn= "<<lon_xm<<" lon & "<<lat_ym<<" lat \n";
  std::cerr<<"destn node = "<<m_Nodes[destn_node].x<<" lon & "<<m_Nodes[destn_node].y<<" lat \n"; 
  Way loc;
  loc.nodes.push_back(destn_node);
  m_Ways.push_back(loc); 
  m_Locations.push_back(Location{m_Ways.size()-1}); 

}

int Model::findNearest(struct Node nd)
{
 auto pos = nodeToVertex.begin();
 int nearest_node = pos->first;
 double min_distance = std::sqrt(std::pow(m_Nodes[pos->first].x - nd.x, 2) + std::pow(m_Nodes[pos->first].y - nd.y, 2));
  
 for(auto &elem: nodeToVertex)
 {
   double distance = std::sqrt(std::pow(m_Nodes[elem.first].x - nd.x, 2) + std::pow(m_Nodes[elem.first].y - nd.y, 2));
   if (min_distance > distance)
   {
     nearest_node = elem.first;
     min_distance = distance;
   }
 }   

 return nearest_node; 
}

double Model::lat2ym(double lat)
{
 const auto pi = 3.14159265358979323846264338327950288;
 const auto deg_to_rad = 2. * pi / 360.;
 const auto earth_radius = 6378137.;
 const auto lat2ymf = [&](double lat) { return log(tan(lat * deg_to_rad / 2 +  pi/4)) / 2 * earth_radius; };
 const auto lon2xmf = [&](double lon) { return lon * deg_to_rad / 2 * earth_radius; };     
 const auto dx = lon2xmf(m_MaxLon) - lon2xmf(m_MinLon);
 const auto dy = lat2ymf(m_MaxLat) - lat2ymf(m_MinLat);
 const auto min_y = lat2ymf(m_MinLat);
 const auto min_x = lon2xmf(m_MinLon);
 m_MetricScale = std::min(dx, dy);

 return (lat2ymf(lat) - min_y) / m_MetricScale;        
}
  
double Model::lon2xm(double lon)
{
 const auto pi = 3.14159265358979323846264338327950288;
 const auto deg_to_rad = 2. * pi / 360.;
 const auto earth_radius = 6378137.;
 const auto lat2ymf = [&](double lat) { return log(tan(lat * deg_to_rad / 2 +  pi/4)) / 2 * earth_radius; };
 const auto lon2xmf = [&](double lon) { return lon * deg_to_rad / 2 * earth_radius; };     
 const auto dx = lon2xmf(m_MaxLon) - lon2xmf(m_MinLon);
 const auto dy = lat2ymf(m_MaxLat) - lat2ymf(m_MinLat);
 const auto min_y = lat2ymf(m_MinLat);
 const auto min_x = lon2xmf(m_MinLon);
 m_MetricScale = std::min(dx, dy);

 return (lon2xmf(lon) - min_x) / m_MetricScale;        
}


void Model::addRoute(std::vector<int> gVertices)
{
  Way route;
  for(auto vert: gVertices)
   route.nodes.push_back(vertexToNode[vert]);
  m_Ways.push_back(route);
  m_Routes.push_back(Route{m_Ways.size()-1});
}
 

