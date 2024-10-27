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
from typing import List
from kant_dto import PddlObjectDto, PddlConditionEffectDto


from yasmin import CbState
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED

from merlin2_fsm_action import Merlin2FsmAction
from merlin2_fsm_action import Merlin2BasicStates
from merlin2_carry_my_luggage.pddl import bag_type, carry, carried_bag
from merlin2_carry_my_luggage.pddl import followed_person, assisted_person
from merlin2_basic_actions.merlin2_basic_types import person_type


VALID = "valid"
REPEAT = "repeat"


class Merlin2OfferBagAction(Merlin2FsmAction):

    def __init__(self):

        self.__bag = PddlObjectDto(bag_type, "b")
        self.__person = PddlObjectDto(person_type, "p")

        super().__init__("offer_bag")

        self.add_state(
            "PREPARING_OFFER_SPEAKING",
            CbState([SUCCEED], self.prepare_offer_speaking),
            transitions={
                SUCCEED: "SPEAKING_OFFER",
            },
        )

        self.add_state(
            "SPEAKING_OFFER",
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
                VALID: "PREPARING_BYE_SPEAKING",
                REPEAT: "PREPARING_OFFER_SPEAKING",
            },
        )

        self.add_state(
            "PREPARING_BYE_SPEAKING",
            CbState([SUCCEED], self.prepare_bye_speaking),
            transitions={
                SUCCEED: "SPEAKING_BYE",
            },
        )

        self.add_state(
            "SPEAKING_BYE",
            self.create_state(Merlin2BasicStates.TTS),
            transitions={
                SUCCEED: SUCCEED,
            },
        )

    def prepare_offer_speaking(self, blackboard: Blackboard) -> str:
        blackboard["text"] = "Please, take your bag. Tell me when it is ready."
        return SUCCEED

    def prepare_bye_speaking(self, blackboard: Blackboard) -> str:
        blackboard["text"] = "Thank you for trusting me, bye bye."
        return SUCCEED

    def check_stt(self, blackboard: Blackboard) -> str:
        if blackboard["speech"][0] == "ready":
            return VALID

        return REPEAT

    ################################
    ############# PDDL #############
    ################################
    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__person, self.__bag]

    def create_conditions(self) -> List[PddlConditionEffectDto]:

        condition_1 = PddlConditionEffectDto(
            carry, [self.__bag], time=PddlConditionEffectDto.AT_START
        )

        condition_2 = PddlConditionEffectDto(
            followed_person, [self.__person], time=PddlConditionEffectDto.AT_START
        )

        return [condition_1, condition_2]

    def create_effects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            carried_bag, [self.__bag], time=PddlConditionEffectDto.AT_END
        )

        effect_2 = PddlConditionEffectDto(
            assisted_person, [self.__person], time=PddlConditionEffectDto.AT_END
        )

        return [effect_1, effect_2]


def main(args=None):
    rclpy.init(args=args)
    node = Merlin2OfferBagAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
