"""Robonix Driver and RPC/MCP facade; start does not start the navigation worker."""

import json
import os
import re
from robonix_api import ATLAS, Service, Ok, Err, Deferred
from outdoor_execution_mcp import (
    FollowReference_Request,
    FollowReference_Response,
    Status_Request,
    Status_Response,
    Cancel_Request,
    Cancel_Response,
    Stop_Request,
    Stop_Response,
)
from .configuration import validate
from .manager import RuntimeManager

service = Service(
    id=os.environ.get("ROBONIX_CAPABILITY_ID", "outdoor_execution"),
    namespace="robonix/service/navigation_outdoor",
)
manager = RuntimeManager()
channels = {}
CONTRACTS = {
    "cloud": "robonix/primitive/lidar/lidar3d",
    "odom": "robonix/service/localization/odom",
    "localization": "robonix/service/localization/status",
}


def resolve(config):
    config = {**config, "topic_remap": dict(config.get("topic_remap", {}))}
    for role, contract in CONTRACTS.items():
        if role in config["topic_remap"]:
            continue
        found = ATLAS.find_capability(
            contract_id=contract,
            transport="ros2",
            provider_id=config.get("provider_ids", {}).get(
                role,
                config.get("provider_ids", {}).get("localization", "")
                if role == "odom"
                else "",
            ),
        )
        if not found:
            return None, "Missing provider for " + role
        channel = service.connect_capability(
            found[0], contract_id=contract, transport="ros2"
        )
        if not channel.endpoint:
            channel.close()
            return None, "Provider endpoint is empty: " + role
        previous = channels.get(role)
        if previous:
            previous.close()
        channels[role] = channel
        config["topic_remap"][role] = channel.endpoint
    if config.get("gait_switching_enabled", False) and "gait_output_topic" not in config:
        contract = "robonix/primitive/chassis/gait_in"
        found = ATLAS.find_capability(contract_id=contract, transport="ros2",
                                     provider_id=config.get("provider_ids", {}).get("chassis", ""))
        if not found:
            return None, "Missing provider for guarded gait input"
        channel = service.connect_capability(found[0], contract_id=contract, transport="ros2")
        if not channel.endpoint:
            channel.close()
            return None, "Guarded gait endpoint is empty"
        previous = channels.get("gait")
        if previous:
            previous.close()
        channels["gait"] = channel
        config["gait_output_topic"] = channel.endpoint
    # JSON localization readiness is deliberately explicit, not guessed from odometry.
    if not config["topic_remap"].get("localization"):
        return None, "topic_remap.localization is required (documented readiness JSON)"
    return config, ""


def close_channels():
    for channel in channels.values():
        channel.close()
    channels.clear()


@service.on_init
def init(config):
    try:
        validate(
            config
        )  # Fail invalid destinations before provider or ROS side effects.
        config, missing = resolve(config)
        if missing:
            close_channels()
            return Deferred(missing)
        manager.start(config)
        service.declare_ros2_topic(
            "robonix/service/navigation_outdoor/costmap",
            "/global_costmap/costmap",
            qos="transient_local",
        )
        return Ok()
    except Exception as e:
        manager.shutdown()
        close_channels()
        return Err(str(e))


@service.on_activate
def activate():
    try:
        manager.active(True)
        return Ok()
    except Exception as e:
        return Err(str(e))


@service.on_deactivate
def deactivate():
    try:
        manager.active(False)
        return Ok()
    except Exception as e:
        return Err(str(e))


@service.on_shutdown
def shutdown():
    manager.shutdown()
    close_channels()
    return Ok()


def follow_reference(req: FollowReference_Request) -> FollowReference_Response:
    """Execute the supplied map-frame reference route with live obstacle avoidance. Does not generate a business route. Retries must reuse the same request_id; arrival means measured stop confirmed."""
    try:
        result = manager.request("/api/tasks", json.loads(req.request_json))
        return FollowReference_Response(
            accepted=True, run_id=result["task_id"], detail="accepted"
        )
    except Exception as e:
        return FollowReference_Response(accepted=False, run_id="", detail=str(e))


def status(req: Status_Request) -> Status_Response:
    """Read execution and readiness. Empty run_id reads the current execution. No velocity is produced by this query."""
    try:
        if req.run_id:
            if not re.fullmatch("[a-f0-9]{32}", req.run_id):
                raise ValueError("Invalid run_id")
            value = manager.request("/api/tasks/" + req.run_id)
            task = value
        else:
            value = manager.request("/api/status")
            task = value.get("task")
        return Status_Response(
            known=task is not None,
            state=task["state"] if task else "idle",
            status_json=json.dumps(value, ensure_ascii=False),
        )
    except Exception as e:
        return Status_Response(
            known=False, state="unavailable", status_json=json.dumps({"error": str(e)})
        )


def cancel(req: Cancel_Request) -> Cancel_Response:
    """Cancel one execution. Acceptance is not a stop confirmation; poll status until terminal."""
    try:
        run = req.run_id
        if not run:
            current = manager.request("/api/status").get("task")
            run = current["task_id"] if current else ""
        if not re.fullmatch("[a-f0-9]{32}", run):
            raise ValueError("No valid current execution")
        manager.request("/api/tasks/" + run + "/cancel", {})
        return Cancel_Response(accepted=True, detail="Stopping; poll status")
    except Exception as e:
        return Cancel_Response(accepted=False, detail=str(e))


def stop(req: Stop_Request) -> Stop_Response:
    """Deactivate motion and wait for a fresh measured standstill. Explicit Driver activation is required before new movement."""
    try:
        manager.active(False)
        return Stop_Response(
            stopped=True, detail="Measured stop confirmed; service inactive"
        )
    except Exception as e:
        return Stop_Response(stopped=False, detail=str(e))


from outdoor_execution_pb2 import (
    FollowReference_Response as FollowPB,
    Status_Response as StatusPB,
    Cancel_Response as CancelPB,
    Stop_Response as StopPB,
)


def grpc_handler(handler, response):
    def invoke(req):
        return response(**handler(req).to_dict())

    return invoke


for name, handler, response in (
    ("follow_reference", follow_reference, FollowPB),
    ("status", status, StatusPB),
    ("cancel", cancel, CancelPB),
    ("stop", stop, StopPB),
):
    contract = "robonix/service/navigation_outdoor/" + name
    service.mcp(contract)(handler)
    service.grpc(contract)(grpc_handler(handler, response))

if __name__ == "__main__":
    service.run()
