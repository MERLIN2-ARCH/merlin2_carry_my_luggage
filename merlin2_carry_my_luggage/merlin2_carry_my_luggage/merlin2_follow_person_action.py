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


import math
import rclpy
from typing import List
from kant_dto import PddlObjectDto, PddlConditionEffectDto

from yasmin import CbState
from yasmin.blackboard import Blackboard
from yasmin_ros import MonitorState, ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT

from merlin2_fsm_action import Merlin2FsmAction
from merlin2_fsm_action import Merlin2BasicStates
from merlin2_basic_actions.merlin2_basic_types import wp_type, person_type
from merlin2_carry_my_luggage.pddl import bag_type, carry, followed_person, person_detected
from merlin2_carry_my_luggage.states import DisplacePoseState

from merlin2_carry_my_luggage_msgs.msg import PointingArray
from waypoint_navigation_msgs.srv import AddWp

NO_NEXT = "not_next"
NO_MOVED = "not_moved"
MOVED = "moved"
YES = "yes"
NO = "not"


class Merlin2FollowPersonAction(Merlin2FsmAction):

    def __init__(self):

        self.__wp = PddlObjectDto(wp_type, "wp")
        self.__bag = PddlObjectDto(bag_type, "b")
        self.__person = PddlObjectDto(person_type, "p")

        super().__init__("follow_person")

        self.add_state(
            "LOOKING_FOR_PERSON",
            MonitorState(
                PointingArray,
                "pointing",
                [SUCCEED, NO_NEXT, CANCEL],
                self.manage_pointing_msgs
            ),
            transitions={
                NO_NEXT: "LOOKING_FOR_PERSON",
                SUCCEED: "CREATING_PERSON_WP",
                CANCEL: CANCEL
            }
        )

        self.add_state(
            "CREATING_PERSON_WP",
            DisplacePoseState(self, distance=0.2),
            transitions={
                SUCCEED: "CHECKING_DISTANCE",
                ABORT: "CREATING_PERSON_WP"
            }
        )

        self.add_state(
            "CHECKING_DISTANCE",
            CbState([MOVED, NO_MOVED], self.check_distance),
            transitions={
                MOVED: "ADDING_PERSON_WP",
                NO_MOVED: "LISTENING"
            }
        )

        self.add_state(
            "LISTENING",
            self.create_state(Merlin2BasicStates.STT),
            transitions={
                SUCCEED: "CHECKING_SPEECH"
            }
        )

        self.add_state(
            "CHECKING_SPEECH",
            CbState([YES, NO], self.check_stt),
            transitions={
                YES: SUCCEED,
                NO: "ADDING_PERSON_WP"
            }
        )

        self.add_state(
            "ADDING_PERSON_WP",
            ServiceState(
                AddWp,
                "/waypoint_navigation/add_wp",
                self.create_addwp_cb,
                timeout=2
            ),
            transitions={
                SUCCEED: "PREPARING_NAVIGATION",
                TIMEOUT: ABORT
            }
        )

        self.add_state(
            "PREPARING_NAVIGATION",
            CbState([SUCCEED], self.preapre_navigation),
            transitions={
                SUCCEED: "NAVIGATING"
            }
        )

        self.add_state(
            "NAVIGATING",
            self.create_state(Merlin2BasicStates.NAVIGATION),
            transitions={
                SUCCEED: "LOOKING_FOR_PERSON"
            }
        )

    def manage_pointing_msgs(self, blackboard: Blackboard, msg: PointingArray) -> str:

        if self.is_canceled():
            return CANCEL

        if msg.pointings:
            blackboard.pointing = msg.pointings[0]
            blackboard.pose = blackboard.pointing.detection.bbox3d.center
            return SUCCEED

        return NO_NEXT

    def check_distance(self, blackboard: Blackboard) -> str:
        pose = blackboard.pose
        displaced_pose = blackboard.displaced_pose

        if math.sqrt(
            math.pow(pose.position.x - displaced_pose.position.x, 2) +
            math.pow(pose.position.y - displaced_pose.position.y, 2)
        ) <= 0.25:
            return NO_MOVED

        return MOVED

    def check_stt(self, blackboard: Blackboard) -> str:
        if blackboard.speech[0] == "yes":
            return YES

        return NO

    def create_addwp_cb(self, blackboard: Blackboard) -> AddWp.Request:
        req = AddWp.Request()
        req.wp.pose = blackboard.displaced_pose
        req.wp.id = "person_wp"
        return req

    def preapre_navigation(self, blackboard: Blackboard) -> str:
        blackboard.destination = "person_wp"
        return SUCCEED

    ################################
    ############# PDDL #############
    ################################
    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__person, self.__bag, self.__wp]

    def create_conditions(self) -> List[PddlConditionEffectDto]:

        condition_1 = PddlConditionEffectDto(
            carry,
            [self.__bag],
            time=PddlConditionEffectDto.AT_START
        )

        condition_2 = PddlConditionEffectDto(
            person_detected,
            [self.__person],
            time=PddlConditionEffectDto.AT_START
        )

        return [condition_1, condition_2]

    def create_effects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            followed_person,
            [self.__person],
            time=PddlConditionEffectDto.AT_END
        )

        return [effect_1]


def main(args=None):
    rclpy.init(args=args)
    node = Merlin2FollowPersonAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
