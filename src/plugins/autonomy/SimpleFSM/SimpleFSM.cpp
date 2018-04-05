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
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::SimpleFSM,
                SimpleFSM_plugin)

namespace scrimmage {
namespace autonomy {

struct DotVertex {
    std::string name; //, label, shape;
};

struct DotEdge {
    std::string label;
    // double weight;
};


typedef boost::property<boost::graph_name_t, std::string> graph_p;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, DotVertex, DotEdge, graph_p> graph_t;

SimpleFSM::SimpleFSM() {
}

void SimpleFSM::init(std::map<std::string, std::string> &params) {
    std::string graph_str = sc::get<std::string>("graphviz_fsm", params, "");

    // Construct an empty graph and prepare the dynamic_property_maps.
    graph_t graph(0);

    boost::dynamic_properties dp(boost::ignore_other_properties);
    dp.property("node_id", boost::get(&DotVertex::name,  graph));
    dp.property("label",   boost::get(&DotEdge::label,   graph));

    // boost::ref_property_map<graph_t*,std::string> gname(boost::get_property(graph, boost::graph_name));
    // dp.property("name", gname);

    if (!boost::read_graphviz(graph_str, graph, dp)) {
        cout << "Failed to parse graphviz fms" << endl;
        return;
    }

    std::cout << "Graph name: '" << boost::get_property(graph, boost::graph_name) << endl;

    const char* name = "ABCDEF";
    cout << "Edge Set:" << endl;
    boost::print_edges(graph, name);
    cout << "Node set: " << endl;
    boost::print_vertices(graph, name);
    cout << "Graph: " << endl;
    boost::print_graph(graph, name);

    // const boost::property_map<graph_t, boost::vertex_attribute_t>::
    //    type& vertAttrMap = boost::get(boost::vertex_attribute, graph);

    typename boost::property_map<graph_t, boost::vertex_index_t>::type vertex_id = boost::get(boost::vertex_index, graph);

    typename boost::property_map<graph_t, boost::vertex_index_t>::type vertex_name = boost::get(boost::vertex_index, graph);


    cout << "iterate over edges..." << endl;
    std::pair<boost::graph_traits<graph_t>::edge_iterator,
              boost::graph_traits<graph_t>::edge_iterator> edgeIteratorRange = boost::edges(graph);
    for (boost::graph_traits<graph_t>::edge_iterator edgeIterator = edgeIteratorRange.first;
        edgeIterator != edgeIteratorRange.second; ++edgeIterator) {
        cout << boost::source(*edgeIterator, graph) << " -> " << boost::target(*edgeIterator, graph) << endl;
        // cout << edgeIterator->label << endl;
        // cout << *ei.first << endl;
        // std::cout << *edgeIterator << ", label=" <<   << std::endl;
    }


    // std::pair<boost::graph_traits<graph_t>::vertex_iterator,
    //           boost::graph_traits<graph_t>::vertex_iterator> vertexIteratorRange = boost::vertices(graph);
    // for (boost::graph_traits<graph_t>::vertex_iterator vertexIterator = vertexIteratorRange.first;
    //      vertexIterator != vertexIteratorRange.second; ++vertexIterator) {
    //     // cout << graph[*vertexIterator].name << endl; // works
    //     boost::graph_traits<graph_t>::vertex_descriptor v = *vertexIterator.first;
    //
    //     //cout << boost::source(*edgeIterator, graph) << " -> " << boost::target(*edgeIterator, graph) << endl;
    //     //cout << *ei.first << endl;
    //     //std::cout << *edgeIterator << ", label=" <<   << std::endl;
    // }

    // boost::ref_property_map<graph_t *, std::string> vertex_struct(boost::get_property(graph, Vertex));
    // dp.property("label");

    // typedef boost::property_map<graph_t, boost::vertex_index_t>::type IndexMap;
    // boost::property_map<graph_t, Vertex>::type vertex_struct;// = boost::get(boost::vertex_index, graph);

    // boost::property_map<graph_t, Vertex>::type vertex_kevin;//
    // = get(edge_weight, g);


    cout << "iterate over vertices..." << endl;
    typedef boost::graph_traits<graph_t>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
        boost::graph_traits<graph_t>::vertex_descriptor v = *vp.first;
        cout << "State Name: " << graph[v].name << endl;
        // std::cout << index[v] <<  " ";
    }

    // // Create a property_map of the input vertex ids
    // boost::property_map<graph_t, Vertex>::type vertex_struct =
    //     boost::get(Vertex(), graph);
    // cout << "Node name: " << graph[0].name << endl;
    // cout << "Node name: " << graph[1].name << endl;

    return;
}

bool SimpleFSM::step_autonomy(double t, double dt) {
    return true;
}
} // namespace autonomy
} // namespace scrimmage
