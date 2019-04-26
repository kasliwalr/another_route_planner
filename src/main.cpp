#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "model.h"
#include "graph.h"
#include "render.h"
#include "http.h"
#include "graph_search.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_bounding_box = "";
    std::string osm_data_file = "";
    int i=0;
    if( argc > 1 ) {
        for(i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-b" && ++i < argc )
            {
                osm_bounding_box = argv[i];
                break;
            }
            else if( std::string_view{argv[i]} == "-f" && ++i < argc )
            {
                osm_data_file = argv[i];
                break;
            }
    }
    else {
        std::cout << "Usage: maps [-b MinLongitude,MinLattitude,MaxLongitude,MaxLattitude] [-f filename.xml]" << std::endl;
        std::cout << "Will use the map of Rapperswil: 8.81598,47.22277,8.83,47.23" << std::endl << std::endl;
        osm_bounding_box = "8.81598,47.22277,8.83,47.23";         
    }
 
    std::vector<std::byte> osm_data;
    
    if( !osm_bounding_box.empty() ) {
        std::cout << "Downloading OpenStreetMap data for the following bounding box: " << osm_bounding_box << std::endl;
        osm_data = FetchOpenStreetMapData( osm_bounding_box );
        if( osm_data.empty() )
            std::cout << "Failed to download." << std::endl;
    }
    
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    
    std::string src_string = "", destn_string = "";
    if (++i < argc)
     src_string = argv[i];
    if (++i < argc)
     destn_string = argv[i];
    
    std::cerr<<"src: "<<src_string<<"-->"<<"destn: "<<destn_string<<std::endl; 
        
    Model model{osm_data};
   
    //rk 
    graph::GraphWAL g(false);  // generate graph from Model
    model.generateGraph(g);
    std::cerr<<"main: generated graph  \n";
    model.addSrcDestn(src_string, destn_string);  // makes 1 node as src and other as destination in Model
    std::cerr<<"main: added src, destn to model \n";
    Modelfunc mem_fn = &Model::eucledianDistance;
    std::vector<int> path = graph::astarSearch(model, mem_fn, g, model.Src(), model.Destn()); // run astar search on graph generated
    
    std::cerr<<"main: completed astar search \n"; 
    model.addRoute(path); // add calculated route to model.m_Routes 
    Render render{model}; // Render model
    

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();  
}
