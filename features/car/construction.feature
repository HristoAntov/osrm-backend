@routing @car @construction
Feature: Car - all construction tags the OpenStreetMap community could think of and then some

    Background:
        Given the profile "car"

    Scenario: Various ways to tag construction and proposed roads
        Then routability should be
            | highway      | construction | proposed | bothw |
            | primary      |              |          | x     |
            | construction |              |          |       |
            | proposed     |              |          |       |
            | primary      | primary      |          |       |
            | primary      |              | primary  |       |
