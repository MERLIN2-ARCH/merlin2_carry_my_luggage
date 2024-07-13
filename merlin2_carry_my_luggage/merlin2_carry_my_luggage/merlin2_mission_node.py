#!/usr/bin/env python3

# Copyright (C) 2024  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import rclpy
from kant_dto import PddlObjectDto
from kant_dto import PddlPropositionDto
from merlin2_mission import Merlin2FsmMissionNode

from threading import Event
from std_srvs.srv import Trigger

from merlin2_basic_actions.merlin2_basic_types import wp_type, person_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at, person_at
from merlin2_carry_my_luggage.pddl import bag_type, bag_at, carried_bag, assisted_person
from merlin2_carry_my_luggage.system_profiler import SystemProfiler

from yasmin import CbState
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT


class Merlin2MissionNode(Merlin2FsmMissionNode):

    def __init__(self) -> None:

        super().__init__(
            "merlin2_mission_node",
            run_mission=False,
            outcomes=[SUCCEED, ABORT]
        )

        self.profiler = SystemProfiler()

        # add states
        self.add_state(
            "WAITING_FOR_START",
            CbState([SUCCEED], self.waiting_for_start),
            transitions={
                SUCCEED: "MOVING_TO_INITIAL_WP"
            }
        )

        self.add_state(
            "MOVING_TO_INITIAL_WP",
            CbState([SUCCEED, ABORT], self.move_to_initial_wp),
            transitions={
                SUCCEED: "RUNNING_MISSION",
                ABORT: ABORT
            }
        )

        self.add_state(
            "RUNNING_MISSION",
            CbState([SUCCEED, ABORT], self.run_carry_mission),
            transitions={
                SUCCEED: "RETURNING_TO_INITIAL_WP",
                ABORT: "RETURNING_TO_INITIAL_WP"
            }
        )

        self.add_state(
            "RETURNING_TO_INITIAL_WP",
            CbState([SUCCEED, ABORT], self.move_to_initial_wp),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: ABORT
            }
        )

        # create services to start the mission
        self.event_start = Event()
        self.srv = self.create_service(
            Trigger, "start_mission", self.start_mission_cb)

    def waiting_for_start(self, blackboard: Blackboard) -> str:
        self.get_logger().info("Waiting for starting")
        self.event_start.clear()
        self.event_start.wait()
        return SUCCEED

    def move_to_initial_wp(self, blackboard: Blackboard) -> str:

        goal = PddlPropositionDto(
            robot_at,
            [self.starting_wp],
            is_goal=True
        )

        if self.execute_goal(goal):
            return SUCCEED
        else:
            return ABORT

    def run_carry_mission(self, blackboard: Blackboard) -> str:

        goal_1 = PddlPropositionDto(
            carried_bag,
            [self.bag],
            is_goal=True
        )

        goal_2 = PddlPropositionDto(
            assisted_person,
            [self.person],
            is_goal=True
        )

        self.profiler.start()
        res = self.execute_goals([goal_1, goal_2])
        self.profiler.stop()
        self.profiler.join()
        self.profiler.save_data()

        if res:
            return SUCCEED
        else:
            return ABORT

    def start_mission_cb(
        self,
        req: Trigger.Request,
        res: Trigger.Response
    ) -> Trigger.Response:
        self.event_start.set()
        res.success = True
        return res

    ################################
    ############# PDDL #############
    ################################
    def create_objects(self) -> rclpy.List[PddlObjectDto]:
        self.person = PddlObjectDto(person_type, "miguel")
        self.bag = PddlObjectDto(bag_type, "my_bag")

        self.person_wp = PddlObjectDto(wp_type, "person_wp")
        self.bag_wp = PddlObjectDto(wp_type, "bag_wp")
        self.starting_wp = PddlObjectDto(wp_type, "starting_wp")
        self.anywhere = PddlObjectDto(wp_type, "anywhere")

        return [
            self.person,
            self.bag,
            self.person_wp,
            self.bag_wp,
            self.starting_wp,
            self.anywhere
        ]

    def create_propositions(self) -> rclpy.List[PddlPropositionDto]:
        robot_at_prop = PddlPropositionDto(robot_at, [self.anywhere])
        person_at_prop = PddlPropositionDto(
            person_at, [self.person, self.starting_wp])
        bag_at_prop = PddlPropositionDto(bag_at, [self.bag, self.bag_wp])
        return [robot_at_prop, person_at_prop, bag_at_prop]


def main():
    rclpy.init()
    node = Merlin2MissionNode()
    node.execute_mission()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
