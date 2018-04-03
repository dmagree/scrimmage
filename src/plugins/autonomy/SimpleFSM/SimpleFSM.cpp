/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/autonomy/SimpleFSM/SimpleFSM.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
// #include <libs/graph/src/read_graphviz_new.cpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::SimpleFSM,
                SimpleFSM_plugin)

namespace scrimmage {
namespace autonomy {

// Vertex properties
//typedef property < vertex_name_t, std::string,
//                   property < vertex_color_t, float > > vertex_p;

//// Edge properties
//typedef property < edge_weight_t, double > edge_p;
//// Graph properties
//typedef property < graph_name_t, std::string > graph_p;
//// adjacency_list-based type
//typedef adjacency_list < vecS, vecS, directedS,
//  vertex_p, edge_p, graph_p > graph_t;

struct Vertex {
    std::string name, label, shape;
};

struct Edge {
    std::string label;
    double weight; // perhaps you need this later as well, just an example
};

typedef boost::property<boost::graph_name_t, std::string> graph_p;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Vertex, Edge, graph_p> graph_t;

//typedef boost::property<boost::graph_name_t, std::string> graph_t;

SimpleFSM::SimpleFSM() {
}

void SimpleFSM::init(std::map<std::string, std::string> &params) {
    std::string graph_str = sc::get<std::string>("graphviz_fsm", params, "");

    // Construct an empty graph and prepare the dynamic_property_maps.
    graph_t graph(0);

    boost::dynamic_properties dp/*(ignore_other_properties)*/;
    dp.property("node_id", boost::get(&Vertex::name,  graph));
    dp.property("label",   boost::get(&Vertex::label, graph));
    dp.property("shape",   boost::get(&Vertex::shape, graph));
    dp.property("label",   boost::get(&Edge::label,   graph));

    std::istringstream graph_stream(graph_str);
    bool status = boost::read_graphviz(graph_stream, graph, dp);
    cout << "STATUS: " << status << endl;
    //return status? 0 : 255;

    //// Use ref_property_map to turn a graph property into a property map
    //std::string graph_name = "FSM";
    //boost::ref_property_map<graph_t *, std::string> gname(boost::get_property(graph, graph_name));
    //dp.property("name",    gname);
}

bool SimpleFSM::step_autonomy(double t, double dt) {
    return true;
}
} // namespace autonomy
} // namespace scrimmage
