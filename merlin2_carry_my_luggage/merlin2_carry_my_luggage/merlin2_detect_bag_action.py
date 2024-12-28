#!/usr/bin/env python3

# Copyright (C) 2024 Miguel Ángel González Santamarta

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
from typing import List
from kant_dto import PddlObjectDto, PddlConditionEffectDto

from yasmin import CbState
from yasmin.blackboard import Blackboard
from yasmin_ros import MonitorState, ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL, TIMEOUT

from merlin2_fsm_action import Merlin2FsmAction
from merlin2_fsm_action import Merlin2BasicStates
from merlin2_basic_actions.merlin2_basic_types import wp_type, person_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at, person_at
from merlin2_carry_my_luggage.pddl import bag_type, bag_detected, person_detected
from merlin2_carry_my_luggage.states import DisplacePoseState

from waypoint_navigation_msgs.srv import AddWp
from merlin2_carry_my_luggage_msgs.msg import PointingArray, Pointing


NEXT = "next"


class Merlin2DetectBagAction(Merlin2FsmAction):

    def __init__(self):

        self.__wp = PddlObjectDto(wp_type, "wp")
        self.__bag = PddlObjectDto(bag_type, "b")
        self.__person = PddlObjectDto(person_type, "p")

        super().__init__("detect_bag")

        self.add_state(
            "PREPARING_POINTING_SPEAKING",
            CbState([SUCCEED], self.prepare_pointing_speaking),
            transitions={
                SUCCEED: "SPEAKING_POINTING",
            },
        )

        self.add_state(
            "SPEAKING_POINTING",
            self.create_state(Merlin2BasicStates.TTS),
            transitions={
                SUCCEED: "LOOKING_FOR_POINTING",
            },
        )

        self.add_state(
            "LOOKING_FOR_POINTING",
            MonitorState(
                PointingArray,
                "pointing",
                [SUCCEED, NEXT, CANCEL],
                self.manage_pointing_msgs,
            ),
            transitions={
                NEXT: "LOOKING_FOR_POINTING",
                SUCCEED: "CREATING_BAG_WP",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "CREATING_BAG_WP",
            DisplacePoseState(self),
            transitions={
                SUCCEED: "ADDING_BAG_WP",
                ABORT: "CREATING_BAG_WP",
            },
        )

        self.add_state(
            "ADDING_BAG_WP",
            ServiceState(
                AddWp, "/waypoint_navigation/add_wp", self.create_addwp_cb, timeout=2
            ),
            transitions={
                SUCCEED: "PREPARING_BAG_SPEAKING",
                TIMEOUT: ABORT,
            },
        )

        self.add_state(
            "PREPARING_BAG_SPEAKING",
            CbState([SUCCEED], self.prepare_bag_speaking),
            transitions={
                SUCCEED: "SPEAKING_BAG",
            },
        )

        self.add_state(
            "SPEAKING_BAG",
            self.create_state(Merlin2BasicStates.TTS),
            transitions={
                SUCCEED: SUCCEED,
            },
        )

    def prepare_pointing_speaking(self, blackboard: Blackboard) -> str:
        blackboard["text"] = "Please, point to bag you want me to carry."
        return SUCCEED

    def manage_pointing_msgs(self, blackboard: Blackboard, msg: PointingArray) -> str:

        if self.is_canceled():
            return CANCEL

        p: Pointing
        for p in msg.pointings:

            if p.direction != -1:
                blackboard["pointing"] = p
                blackboard["pose"] = blackboard["pointing"].detection.bbox3d.center
                return SUCCEED

        return NEXT

    def prepare_bag_speaking(self, blackboard: Blackboard) -> str:

        side = "left"

        if blackboard["pointing"].direction == 1:
            side = "right"
        elif blackboard["pointing"].direction == 0:
            side = "left"

        blackboard["text"] = f"I will carry you {side} bag."
        return SUCCEED

    def create_addwp_cb(self, blackboard: Blackboard) -> AddWp.Request:
        req = AddWp.Request()
        req.wp.pose = blackboard["displaced_pose"]
        req.wp.id = "bag_wp"
        return req

    ################################
    ############# PDDL #############
    ################################
    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__person, self.__bag, self.__wp]

    def create_conditions(self) -> List[PddlConditionEffectDto]:

        condition_1 = PddlConditionEffectDto(
            robot_at, [self.__wp], time=PddlConditionEffectDto.AT_START
        )

        condition_2 = PddlConditionEffectDto(
            person_at, [self.__person, self.__wp], time=PddlConditionEffectDto.AT_START
        )

        return [condition_1, condition_2]

    def create_effects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            bag_detected, [self.__bag], time=PddlConditionEffectDto.AT_END
        )

        effect_2 = PddlConditionEffectDto(
            person_detected, [self.__person], time=PddlConditionEffectDto.AT_END
        )

        return [effect_1, effect_2]


def main(args=None):
    rclpy.init(args=args)
    node = Merlin2DetectBagAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
