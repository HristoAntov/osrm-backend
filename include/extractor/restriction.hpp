#ifndef RESTRICTION_HPP
#define RESTRICTION_HPP

#include "util/coordinate.hpp"
#include "util/opening_hours.hpp"
#include "util/typedefs.hpp"

#include <boost/variant.hpp>
#include <limits>

namespace osrm
{
namespace extractor
{

// OSM offers two types of restrictions, via node and via-way restrictions. We parse both into the
// same input container
//
// A restriction turning at a single node. This is the most common type of restriction:
//
// a - b - c
//     |
//     d
//
// ab via b to bd
struct InputNodeRestriction
{
    OSMEdgeID_weak from;
    OSMNodeID_weak via;
    OSMEdgeID_weak to;
};

// A restriction that uses a single via-way in between
//
// f - e - d
//     |
// a - b - c
//
// ab via be to ef -- no u turn
struct InputWayRestriction
{
    OSMEdgeID_weak from;
    OSMEdgeID_weak via;
    OSMEdgeID_weak to;
};

// Outside view of the variant
enum class RestrictionType : std::uint8_t
{
    NODE_RESTRICTION,
    WAY_RESTRICTION
};

namespace restriction_details
{
struct Bits
{ // mostly unused, initialised to false by default
    Bits() : is_only(false), way_type(false), unused(false) {}

    bool is_only : 1;
    bool way_type : 1; // if false -> node_type
    bool unused : 6;
};

} // namespace restriction

struct InputTurnRestriction
{
    boost::variant<InputWayRestriction, InputNodeRestriction> node_or_way;
    restriction_details::Bits flags;

    OSMEdgeID_weak from() const
    {
        return flags.way_type ? boost::get<InputWayRestriction>(node_or_way).from
                              : boost::get<InputNodeRestriction>(node_or_way).from;
    }

    OSMEdgeID_weak to() const
    {
        return flags.way_type ? boost::get<InputWayRestriction>(node_or_way).to
                              : boost::get<InputNodeRestriction>(node_or_way).to;
    }

    RestrictionType type() const
    {
        return flags.way_type ? RestrictionType::WAY_RESTRICTION
                              : RestrictionType::NODE_RESTRICTION;
    }

    InputWayRestriction &asWayRestriction()
    {
        BOOST_ASSERT(flags.way_type);
        return boost::get<InputWayRestriction>(node_or_way);
    }

    const InputWayRestriction &asWayRestriction() const
    {
        BOOST_ASSERT(flags.way_type);
        return boost::get<InputWayRestriction>(node_or_way);
    }

    InputNodeRestriction &asNodeRestriction()
    {
        BOOST_ASSERT(!flags.way_type);
        return boost::get<InputNodeRestriction>(node_or_way);
    }

    const InputNodeRestriction &asNodeRestriction() const
    {
        BOOST_ASSERT(!flags.way_type);
        return boost::get<InputNodeRestriction>(node_or_way);
    }
};
struct InputConditionalTurnRestriction : InputTurnRestriction
{
    std::vector<util::OpeningHours> condition;
};

// OSRM manages restrictions based on node IDs which refer to the last node along the edge. Note
// that this has the side-effect of not allowing parallel edges!
struct NodeRestriction
{
    NodeID from;
    NodeID via;
    NodeID to;

    // check if all parts of the restriction reference an actual node
    bool valid() const
    {
        return from != SPECIAL_NODEID && to != SPECIAL_NODEID && via != SPECIAL_NODEID;
    };

    std::string toString() const
    {
        return "From " + std::to_string(from) + " via " + std::to_string(via) + " to " +
               std::to_string(to);
    }
};

// A way restriction in the context of OSRM requires translation into NodeIDs. This is due to the
// compression happening in the graph creation process which would make it difficult to track
// way-ids over a series of operations. Having access to the nodes directly allows look-up of the
// edges in the processed structures
struct WayRestriction
{
    // a way restriction in OSRM is essentially a dual node turn restriction;
    //
    // |     |
    // c -x- b
    // |     |
    // d     a
    //
    // from ab via bxc to cd: no_uturn
    //
    // Technically, we would need only a,b,c,d to describe the full turn in terms of nodes. When
    // parsing the relation, though, we do not know about the final representation in the node-based
    // graph for the restriction. In case of a traffic light, for example, we might end up with bxc
    // not being compressed to bc. For that reason, we need to maintain two node restrictions in
    // case a way restrction is not fully collapsed
    NodeRestriction in_restriction;
    NodeRestriction out_restriction;
};

// Wrapper for turn restrictions that gives more information on its type / handles the switch
// between node/way/multi-way restrictions
struct TurnRestriction
{
    boost::variant<NodeRestriction, WayRestriction> node_or_way;
    restriction_details::Bits flags;

    // construction for NodeRestrictions
    explicit TurnRestriction(NodeRestriction node_restriction, bool is_only = false)
        : node_or_way(node_restriction)
    {
        flags.way_type = false;
        flags.is_only = is_only;
    }

    // construction for WayRestrictions
    explicit TurnRestriction(WayRestriction way_restriction, bool is_only = false)
        : node_or_way(way_restriction)
    {
        flags.way_type = true;
        flags.is_only = is_only;
    }

    explicit TurnRestriction()
    {
        node_or_way = NodeRestriction{SPECIAL_EDGEID, SPECIAL_NODEID, SPECIAL_EDGEID};
    }

    // Sentinel for comparison
    static TurnRestriction min_value() { return TurnRestriction(NodeRestriction{0, 0, 0}); }

    // Sentinel for comparison
    static TurnRestriction max_value()
    {
        return TurnRestriction(WayRestriction{{SPECIAL_NODEID, SPECIAL_NODEID, SPECIAL_NODEID},
                                              {SPECIAL_NODEID, SPECIAL_NODEID, SPECIAL_NODEID}});
    }

    WayRestriction &asWayRestriction()
    {
        BOOST_ASSERT(flags.way_type);
        return boost::get<WayRestriction>(node_or_way);
    }

    const WayRestriction &asWayRestriction() const
    {
        BOOST_ASSERT(flags.way_type);
        return boost::get<WayRestriction>(node_or_way);
    }

    NodeRestriction &asNodeRestriction()
    {
        BOOST_ASSERT(!flags.way_type);
        return boost::get<NodeRestriction>(node_or_way);
    }

    const NodeRestriction &asNodeRestriction() const
    {
        BOOST_ASSERT(!flags.way_type);
        return boost::get<NodeRestriction>(node_or_way);
    }

    RestrictionType type() const
    {
        return flags.way_type ? RestrictionType::WAY_RESTRICTION
                              : RestrictionType::NODE_RESTRICTION;
    }

    // check if all elements of the edge are considered valid
    bool valid() const
    {
        if (flags.way_type)
        {
            auto const &restriction = asWayRestriction();
            return restriction.in_restriction.valid() && restriction.out_restriction.valid();
        }
        else
        {
            auto const &restriction = asNodeRestriction();
            return restriction.valid();
        }
    }

    std::string toString() const
    {
        std::string representation;
        if (flags.way_type)
        {
            auto const &way = asWayRestriction();
            representation =
                "In: " + way.in_restriction.toString() + " Out: " + way.out_restriction.toString();
        }
        else
        {
            auto const &node = asNodeRestriction();
            representation = node.toString();
        }
        representation += " is_only: " + std::to_string(flags.is_only);
        return representation;
    }
};

struct ConditionalTurnRestriction : TurnRestriction
{
    std::vector<util::OpeningHours> condition;
};
}
}

#endif // RESTRICTION_HPP
