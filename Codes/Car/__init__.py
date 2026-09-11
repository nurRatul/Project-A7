__all__ = ["CarController", "BTSMotor"]


def __getattr__(name):
	if name == "CarController":
		from .carController import CarController
		return CarController
	if name == "BTSMotor":
		from .btsMotor import BTSMotor
		return BTSMotor
	raise AttributeError(f"module {__name__!r} has no attribute {name!r}")