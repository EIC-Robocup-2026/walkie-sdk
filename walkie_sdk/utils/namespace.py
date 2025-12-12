"""
Namespace - ROS namespace utilities.

Provides functions for applying ROS namespace prefixes to topic and action names.
"""


def apply_namespace(name: str, namespace: str) -> str:
    """
    Apply ROS namespace prefix to a topic/action name.

    Args:
        name: Topic or action name without leading slash (e.g., "odom", "cmd_vel")
        namespace: Namespace prefix (e.g., "robot1", "walkie"). Empty string for no namespace.

    Returns:
        Full topic/action name with namespace (e.g., "/robot1/odom", "/cmd_vel")

    Example:
        >>> apply_namespace("odom", "")
        '/odom'
        >>> apply_namespace("odom", "robot1")
        '/robot1/odom'
        >>> apply_namespace("cmd_vel", "my_robot")
        '/my_robot/cmd_vel'
    """
    if namespace:
        # Ensure namespace doesn't have leading/trailing slashes
        ns = namespace.strip("/")
        return f"{ns}/{name}"
    else:
        return f"{name}"
