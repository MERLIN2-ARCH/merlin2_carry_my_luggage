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
from yasmin_ros.basic_outcomes import SUCCEED

from merlin2_fsm_action import Merlin2FsmAction
from merlin2_fsm_action import Merlin2BasicStates
from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at
from merlin2_carry_my_luggage.pddl import bag_type, bag_detected, carry, bag_at


VALID = "valid"
REPEAT = "repeat"


class Merlin2PickBagAction(Merlin2FsmAction):

    def __init__(self):

        self.__wp = PddlObjectDto(wp_type, "wp")
        self.__bag = PddlObjectDto(bag_type, "b")

        super().__init__("pick_bag")

        self.add_state(
            "PREPARING_CARRY_SPEAKING",
            CbState([SUCCEED], self.prepare_carry_speaking),
            transitions={
                SUCCEED: "SPEAKING_CARRY",
            },
        )

        self.add_state(
            "SPEAKING_CARRY",
            self.create_state(Merlin2BasicStates.TTS),
            transitions={
                SUCCEED: "LISTENING",
            },
        )

        self.add_state(
            "LISTENING",
            self.create_state(Merlin2BasicStates.STT),
            transitions={
                SUCCEED: "CHECKING_SPEECH",
            },
        )

        self.add_state(
            "CHECKING_SPEECH",
            CbState([VALID, REPEAT], self.check_stt),
            transitions={
                VALID: "PREPARING_FOLLOW_SPEAKING",
                REPEAT: "PREPARING_CARRY_SPEAKING",
            },
        )

        self.add_state(
            "PREPARING_FOLLOW_SPEAKING",
            CbState([SUCCEED], self.prepare_follow_speaking),
            transitions={
                SUCCEED: "SPEAKING_FOLLOW",
            },
        )

        self.add_state(
            "SPEAKING_FOLLOW",
            self.create_state(Merlin2BasicStates.TTS),
            transitions={
                SUCCEED: SUCCEED,
            },
        )

    def prepare_carry_speaking(self, blackboard: Blackboard) -> str:
        blackboard["text"] = (
            "Please, give me your bag, I will carry for you. Tell me when it is ready."
        )
        return SUCCEED

    def prepare_follow_speaking(self, blackboard: Blackboard) -> str:
        blackboard["text"] = "Go on, I will follow you."
        return SUCCEED

    def check_stt(self, blackboard: Blackboard) -> str:
        if blackboard["speech"][0] == "ready":
            return VALID

        return REPEAT

    ################################
    ############# PDDL #############
    ################################
    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__bag, self.__wp]

    def create_conditions(self) -> List[PddlConditionEffectDto]:

        condition_1 = PddlConditionEffectDto(
            robot_at, [self.__wp], time=PddlConditionEffectDto.AT_START
        )

        condition_2 = PddlConditionEffectDto(
            bag_at, [self.__bag, self.__wp], time=PddlConditionEffectDto.AT_START
        )

        condition_3 = PddlConditionEffectDto(
            bag_detected, [self.__bag], time=PddlConditionEffectDto.AT_START
        )

        return [condition_1, condition_2, condition_3]

    def create_effects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            carry, [self.__bag], time=PddlConditionEffectDto.AT_END
        )

        effect_2 = PddlConditionEffectDto(
            bag_at,
            [self.__bag, self.__wp],
            time=PddlConditionEffectDto.AT_END,
            is_negative=True,
        )

        return [effect_1, effect_2]


def main(args=None):
    rclpy.init(args=args)
    node = Merlin2PickBagAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
