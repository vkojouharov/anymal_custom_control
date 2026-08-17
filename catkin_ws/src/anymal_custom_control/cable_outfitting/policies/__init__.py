from .home import CablePolicy as HomePolicy, PolicyCommand as HomeCommand, PolicyObservation as HomeObservation
from .hook import CablePolicy as HookPolicy, PolicyCommand as HookCommand, PolicyObservation as HookObservation
from .pick import CablePolicy as PickPolicy, PolicyCommand as PickCommand, PolicyObservation as PickObservation
from .place import CablePolicy as PlacePolicy, PolicyCommand as PlaceCommand, PolicyObservation as PlaceObservation


POLICY_REGISTRY = {
    "home": (HomePolicy, HomeObservation, HomeCommand),
    "pick": (PickPolicy, PickObservation, PickCommand),
    "place": (PlacePolicy, PlaceObservation, PlaceCommand),
    "hook": (HookPolicy, HookObservation, HookCommand),
}


__all__ = ["POLICY_REGISTRY"]
