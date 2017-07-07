#ifndef EXTRACTION_CONTAINERS_HPP
#define EXTRACTION_CONTAINERS_HPP

#include "extractor/first_and_last_segment_of_way.hpp"
#include "extractor/guidance/turn_lane_types.hpp"
#include "extractor/internal_extractor_edge.hpp"
#include "extractor/query_node.hpp"
#include "extractor/restriction.hpp"
#include "extractor/scripting_environment.hpp"

#include "storage/io.hpp"

#include <cstdint>
#include <stxxl/vector>
#include <unordered_map>

// TODO: move to CMake
//#define USE_STXXL_IN_EXTRACT 1

namespace osrm
{
namespace extractor
{

/**
 * Uses external memory containers from stxxl to store all the data that
 * is collected by the extractor callbacks.
 *
 * The data is the filtered, aggregated and finally written to disk.
 */
class ExtractionContainers
{
#if USE_STXXL_IN_EXTRACT
    template <typename T> using ExtractionVector = stxxl::vector<T>;
#else
    template <typename T> using ExtractionVector = std::vector<T>;
#endif

    void FlushVectors();
    void PrepareNodes();
    void PrepareRestrictions();
    void PrepareEdges(ScriptingEnvironment &scripting_environment);

    void WriteNodes(storage::io::FileWriter &file_out) const;
    void WriteRestrictions(const std::string &restrictions_file_name);
    void WriteEdges(storage::io::FileWriter &file_out) const;
    void WriteCharData(const std::string &file_name);

  public:
    using NodeIDVector = ExtractionVector<OSMNodeID>;
    using NodeVector = ExtractionVector<QueryNode>;
    using EdgeVector = ExtractionVector<InternalExtractorEdge>;
    using RestrictionsVector = ExtractionVector<InputRestrictionContainer>;
    using WayIDStartEndVector = ExtractionVector<FirstAndLastSegmentOfWay>;
    using NameCharData = ExtractionVector<unsigned char>;
    using NameOffsets = ExtractionVector<unsigned>;

    std::vector<OSMNodeID> barrier_nodes;
    std::vector<OSMNodeID> traffic_lights;
    NodeIDVector used_node_id_list;
    NodeVector all_nodes_list;
    EdgeVector all_edges_list;
    NameCharData name_char_data;
    NameOffsets name_offsets;
    // an adjacency array containing all turn lane masks
    RestrictionsVector restrictions_list;
    WayIDStartEndVector way_start_end_id_list;
    std::unordered_map<OSMNodeID, NodeID> external_to_internal_node_id_map;
    unsigned max_internal_node_id;
    std::vector<TurnRestriction> unconditional_turn_restrictions;

    ExtractionContainers();

    void PrepareData(ScriptingEnvironment &scripting_environment,
                     const std::string &output_file_name,
                     const std::string &restrictions_file_name,
                     const std::string &names_file_name);
};
}
}

#endif /* EXTRACTION_CONTAINERS_HPP */
