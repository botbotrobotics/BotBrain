#include <algorithm> 
#include "graph_node.hpp"

GraphNode::GraphNode(const std::string& json_path) : json_path_(json_path) {}

GraphNode::~GraphNode(){}

std::tuple<std::vector<NodeProfile>, bool> GraphNode::load_parameters()
{
    std::vector<NodeProfile> nodes;
    nodes.clear();

    const std::filesystem::path p(json_path_);

    try
    {
        bool any_loaded = false;

        if (!std::filesystem::is_directory(p))
        {
            std::cerr << "[GraphNode] Path '" << p.string() << "' is not a directory.\n";
            return {nodes, false};
        }

        for (const auto& entry : std::filesystem::directory_iterator(p))
        {
            if (!entry.is_regular_file()) continue;
            const auto& f = entry.path();
            if (!f.has_extension() || f.extension() != ".json") continue;

            std::ifstream ifs(f);
            if (!ifs.is_open())
            {
                std::cerr << "[GraphNode] Could not open JSON file '" << f.string() << "'.\n";
                continue;
            }

            nlohmann::json doc;
            try
            {
                ifs >> doc;
            }
            catch (const std::exception& e)
            {
                std::cerr << "[GraphNode] Failed to parse '" << f.string() << "': " << e.what() << "\n";
                continue;
            }

            if (parse_nodes(doc, nodes))
            {
                any_loaded = true;
            }
            else
            {
                std::cerr << "[GraphNode] No valid nodes found in '" << f.filename().string() << "'.\n";
            }
        }
        if (!any_loaded)
        {
            std::cerr << "[GraphNode] No JSON definitions produced valid nodes.\n";
            return {nodes, false};
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[GraphNode] Exception while iterating directory: " << e.what() << "\n";
        return {nodes, false};
    }
    catch (...)
    {
        std::cerr << "[GraphNode] Unknown error while reading node directory.\n";
        return {nodes, false};
    }

    if (!build_dependents(nodes)) return {nodes, false};
    sort_nodes(nodes);
    return {std::move(nodes), true};
}

bool GraphNode::build_dependents(std::vector<NodeProfile>& nodes)
{
    for (auto& np : nodes) {np.dependents_of.clear();}

    for (const auto& np : nodes) 
    {
        for (const auto& dep : np.dependencies)
        {
            auto it = std::find_if(nodes.begin(), nodes.end(),
                [&](const NodeProfile& n){ return n.name == dep; });
            if (it == nodes.end())
            {
                std::cerr << "[GraphNode] Dependency '" << dep << "' declared by '"
                    << np.name << "' does not exist.\n";
                return false;
            }
            if (dep == np.name)
            {
                std::cerr << "[GraphNode] Node '" << np.name << "' cannot depend on itself.\n";
                return false;
            }

            // Avoid duplicates when linking dependents.
            auto& vec = it->dependents_of;
            if (std::find(vec.begin(), vec.end(), np.name) == vec.end()) {vec.push_back(np.name);}
        }
    }

    return true;
}

bool GraphNode::parse_nodes(const nlohmann::json& doc, std::vector<NodeProfile>& nodes)
{
    if (!doc.contains("nodes") || !doc["nodes"].is_array())
    {
        std::cerr << "[GraphNode] JSON document missing 'nodes' array.\n";
        return false;
    }

    for (const auto &item : doc["nodes"])
    {
        if(!item.contains("name") || !item["name"].is_string()) {continue;}
        if(!item.contains("display_name") || !item["display_name"].is_string()) {continue;}
        if(!item.contains("classe") || !item["classe"].is_string()) {continue;}
        if(!item.contains("order") || !item["order"].is_number_integer()) {continue;}

        NodeProfile np;
        np.name = item["name"].get<std::string>();
        np.display_name = item["display_name"].get<std::string>();
        std::string classe = item["classe"].get<std::string>();
        np.classe = parse_classe(classe);
        np.order = item["order"].get<int>();
        np.priority = classe_priority(np.classe);

        if(item.contains("dependencies") && item["dependencies"].is_array())
        {
            for (const auto& dep : item["dependencies"])
            {
                if (dep.is_string())
                    np.dependencies.push_back(dep.get<std::string>());
            }
        }

        nodes.push_back(std::move(np));
    }

    if (nodes.empty())
    {
        std::cerr << "[GraphNode] Document contained no parsable nodes.\n";
        return false;
    }

    return true;
}

void GraphNode::sort_nodes(std::vector<NodeProfile>& nodes)
{
    std::sort(nodes.begin(), nodes.end(),
        [](const NodeProfile& a, const NodeProfile& b)
        {
            // First criterion: class order (enum value encodes severity).
            if (a.classe != b.classe)
                return static_cast<uint8_t>(a.classe) < static_cast<uint8_t>(b.classe);

            // Second criterion: execution order within the same class.
            return a.order < b.order;
        }
    );
}

NodeClasse GraphNode::parse_classe(const std::string& s) 
{
    std::string t = s;                                               
    std::transform(t.begin(), t.end(), t.begin(), 
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); }); 
    if (t == "core") return NodeClasse::CORE;
    if (t == "navigation") return NodeClasse::NAVIGATION;
    if (t == "audio") return NodeClasse::AUDIO;
    if (t == "ia_stack") return NodeClasse::IA_STACK;
    if (t == "payload") return NodeClasse::PAYLOAD;
    if (t == "accessories") return NodeClasse::ACCESSORIES;
    if (t == "camera") return NodeClasse::CAMERA;
    return NodeClasse::UNKNOWN;
}


ClassePriority GraphNode::classe_priority(NodeClasse c) 
{
    switch (c) 
    {
        case NodeClasse::CORE:        return ClassePriority::TERMINAL;
        case NodeClasse::NAVIGATION:  return ClassePriority::WARNING;
        case NodeClasse::AUDIO:       return ClassePriority::WARNING;
        case NodeClasse::IA_STACK:    return ClassePriority::WARNING;
        case NodeClasse::PAYLOAD:     return ClassePriority::WARNING;
        case NodeClasse::ACCESSORIES: return ClassePriority::WARNING;
        case NodeClasse::CAMERA: return ClassePriority::WARNING;
        default: return ClassePriority::UNKNOWN;
    }
}