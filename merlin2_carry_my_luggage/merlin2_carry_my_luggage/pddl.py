

from merlin2_basic_actions.merlin2_basic_types import wp_type, person_type
from kant_dto import PddlPredicateDto, PddlTypeDto


# Predicates
bag_type = PddlTypeDto("bag")
bag_at = PddlPredicateDto("bag_at", [bag_type, wp_type])
bag_detected = PddlPredicateDto("bag_detected", [bag_type])
carry = PddlPredicateDto("carry", [bag_type])
carried_bag = PddlPredicateDto("carried_bag", [bag_type])
pointed_to = PddlPredicateDto("pointed_to", [person_type, bag_type])

followed_person = PddlPredicateDto("followed_person", [person_type])
person_detected = PddlPredicateDto("person_detected", [person_type])
assisted_person = PddlPredicateDto("assisted_person", [person_type])
