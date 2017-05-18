Feature: Profile API version 2

    Background:
        Given a grid size of 100 meters

    Scenario: Basic profile function calls and property values
        Given the profile file
            """
            require('lib/profile_v2')

            function setup()
              return {
                max_speed_for_map_matching      = 180/3.6,
                use_turn_restrictions           = true,
                continue_straight_at_waypoint   = true,
                weight_name                     = 'test_version2',
                weight_precision                = 2
              }
            end

            function node_function (profile, node, result)
              print ('node_function ' .. node:id())
            end

            function way_function(profile, way, result)
              result.name = way:get_value_by_key('name')
              result.weight = 10
              result.forward_mode = mode.driving
              result.backward_mode = mode.driving
              result.forward_speed = 36
              result.backward_speed = 36
              print ('way_function ' .. way:id() .. ' ' .. result.name)
            end

            function turn_function (profile, turn)
              print('turn_function', turn.angle, turn.turn_type, turn.direction_modifier, turn.has_traffic_light)
              turn.weight = turn.angle == 0 and 0 or 4.2
              turn.duration = turn.weight
            end

            function segment_function (profile, segment)
                print ('segment_function ' .. segment.source.lon .. ' ' .. segment.source.lat)
            end

            return {
              setup = setup,
              node = node_function,
              way = way_function,
              segment = segment_function,
              turn = turn_function
            }
            """
        And the node map
            """
               a
              bcd
               e
            """
        And the ways
            | nodes  |
            | ac     |
            | cb     |
            | cd     |
            | ce     |
        And the data has been saved to disk

        When I run "osrm-extract --profile {profile_file} {osm_file}"
        Then it should exit successfully
        And stdout should contain "node_function"
        And stdout should contain "way_function"
        And stdout should contain "turn_function"
        And stdout should contain "segment_function"

        When I route I should get
           | from | to | route    | time  |
           | a    | b  | ac,cb,cb | 19.2s |
           | a    | d  | ac,cd,cd | 19.2s |
           | a    | e  | ac,ce    | 20s   |
