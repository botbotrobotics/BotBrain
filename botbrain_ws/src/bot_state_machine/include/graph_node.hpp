#pragma once

#include <tuple>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <filesystem>

#include "types.hpp"
#include "nlohmann/json.hpp"

/**
 * @brief Reads the state-machine graph from JSON and exposes helpers that
 * transform it into runtime NodeProfile structures.
 */
class GraphNode
{
public:
    /**
     * @brief Construct the parser bound to a JSON definition file.
     * @param json_path Absolute or relative path to the JSON file.
     */
    explicit GraphNode(const std::string& json_path);

    // Destructor.
    ~GraphNode();

public:
    // Parse the JSON file and return the discovered nodes.
    std::tuple<std::vector<NodeProfile>, bool> load_parameters();

private:

    // Path to the JSON file used across helper functions.
    std::string json_path_;

    // Build dependency lists between already-parsed nodes.
    static bool build_dependents(std::vector<NodeProfile>& nodes);

    // Convert the raw JSON document into NodeProfile structures.
    static bool parse_nodes(const nlohmann::json& doc, std::vector<NodeProfile>& nodes);
    
    // Sort parsed nodes by class/priority for deterministic execution.
    static void sort_nodes(std::vector<NodeProfile>& nodes);

    // Translate a JSON class field into a NodeClasse enum value.
    static NodeClasse parse_classe(const std::string& s);

    // Return the priority for a given NodeClasse.
    static ClassePriority classe_priority(NodeClasse c);  
}; 
