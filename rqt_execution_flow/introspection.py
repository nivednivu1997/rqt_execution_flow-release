"""Helper to turn on service introspection in your own rclpy nodes.

On ROS 2 Jazzy, service calls are only visible to tools if the node that makes
or serves the call turns on "service introspection".  Call this once, after you
have created your clients and services::

    from rqt_execution_flow.introspection import enable_service_introspection
    enable_service_introspection(node)

C++ equivalent, per client or service::

    client->configure_introspection(
        node->get_clock(), rclcpp::SystemDefaultsQoS(),
        RCL_SERVICE_INTROSPECTION_CONTENTS);
"""

from rclpy.qos import qos_profile_system_default
from rclpy.service_introspection import ServiceIntrospectionState

# Standard services every node has. They are noise for this tool.
PARAMETER_SERVICE_SUFFIXES = (
    '/describe_parameters',
    '/get_parameter_types',
    '/get_parameters',
    '/list_parameters',
    '/set_parameters',
    '/set_parameters_atomically',
    '/get_type_description',
)


def is_internal_service(name: str) -> bool:
    """Return True for parameter services and the hidden services inside actions."""
    return '/_action/' in name or name.endswith(PARAMETER_SERVICE_SUFFIXES)


def enable_service_introspection(node, include_contents: bool = True,
                                 qos=qos_profile_system_default) -> int:
    """Enable introspection on every client and service of ``node``.

    :param include_contents: also publish request/response data, not only timing.
    :returns: number of clients and services configured.
    """
    state = (ServiceIntrospectionState.CONTENTS if include_contents
             else ServiceIntrospectionState.METADATA)
    count = 0
    for client in node.clients:
        if not is_internal_service(client.srv_name):
            client.configure_introspection(node.get_clock(), qos, state)
            count += 1
    for service in node.services:
        if not is_internal_service(service.srv_name):
            service.configure_introspection(node.get_clock(), qos, state)
            count += 1
    return count
