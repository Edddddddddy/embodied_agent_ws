from copy import deepcopy

from .types import ActionCommand


class ActionValidator:
    """Allow-list and range checks before commands reach robot control."""

    ALLOWED = {
        "stop": {},
        "move": {"linear_x": (-0.5, 0.5), "duration_s": (0.0, 10.0)},
        "turn": {"angular_z": (-1.5, 1.5), "duration_s": (0.0, 10.0)},
        "wave": {"count": (1, 5)},
        "set_led": {},
    }

    def validate(self, command: ActionCommand) -> ActionCommand:
        if command.name not in self.ALLOWED:
            raise ValueError(f"unsupported action: {command.name}")
        arguments = deepcopy(command.arguments)
        for key, bounds in self.ALLOWED[command.name].items():
            if key not in arguments:
                continue
            value = arguments[key]
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise ValueError(f"{key} must be numeric")
            lower, upper = bounds
            arguments[key] = max(lower, min(upper, value))
        return ActionCommand(command.name, arguments)

