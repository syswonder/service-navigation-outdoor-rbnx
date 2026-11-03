"""Explicit-endpoint gRPC client for the external business/UI process."""

import json
import grpc
import outdoor_execution_pb2 as messages
import robonix_contracts_pb2_grpc as contracts


class ExecutionClient:
    def __init__(self, endpoint):
        self.channel = grpc.insecure_channel(endpoint)
        self.follow = contracts.RobonixServiceNavigationOutdoorFollowReferenceStub(
            self.channel
        )
        self.read = contracts.RobonixServiceNavigationOutdoorStatusStub(self.channel)
        self.cancel_rpc = contracts.RobonixServiceNavigationOutdoorCancelStub(
            self.channel
        )

    def follow_reference(self, request):
        result = self.follow.FollowReference(
            messages.FollowReference_Request(request_json=json.dumps(request)),
            timeout=10,
        )
        if not result.accepted:
            raise ValueError(result.detail)
        return result.run_id

    def status(self, run_id=""):
        result = self.read.Status(messages.Status_Request(run_id=run_id), timeout=5)
        value = json.loads(result.status_json)
        if result.state == "unavailable":
            raise ValueError(value.get("error", "Service unavailable"))
        return value

    def cancel(self, run_id=""):
        result = self.cancel_rpc.Cancel(
            messages.Cancel_Request(run_id=run_id), timeout=5
        )
        if not result.accepted:
            raise ValueError(result.detail)
        return self.status(run_id)

    def close(self):
        self.channel.close()
