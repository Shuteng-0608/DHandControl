"""Serialized actuator I/O, with no device construction or automatic reconnect.

The teleop runner defaults to simulation; live send-only testing explicitly
attaches a sender and disables feedback. Feedback timestamps describe device queries, not AVP.
"""
import copy
import math
import time

from mh6_actuator_preview import prepare_packet
from mh6_hardware_config import CHANNELS, FINGERS


class FeedbackPoller:
    """Read one of nine channels per slot; never claim cached data is a new read."""
    def __init__(self, driver=None, query_rate=10., clock=time.monotonic):
        if not math.isfinite(query_rate) or query_rate <= 0:
            raise ValueError("feedback query rate must be positive and finite")
        self.driver, self.clock = driver, clock
        self.period = 1./query_rate
        self.next_query = 0.
        self.index = 0
        self.samples = {name: {"last_attempt": None, "last_valid": None} for name in CHANNELS}

    def poll(self):
        now = self.clock()
        if self.driver is None or now < self.next_query:
            return None
        name = CHANNELS[self.index]
        self.index = (self.index+1) % len(CHANNELS)
        device_id = int(name[1:])
        started = now
        try:
            method = self.driver.read_finger_status if name in FINGERS else self.driver.read_palm_status
            data = method(device_id)
            valid = (isinstance(data, dict) and type(data.get("id")) is int and data.get("id") == device_id
                     and data.get("query_ok") is True and data.get("checksum_ok") is True
                     and data.get("firmware_status") == (0x93 if name in FINGERS else 0x94)
                     and data.get("response_error_code") == 0
                     and type(data.get("current_position")) is int)
            error = None if valid else "INVALID_OR_MISSING_FEEDBACK"
        except Exception as exc:
            data, valid, error = None, False, f"{type(exc).__name__}: {exc}"
        ended = self.clock()
        sample = {"channel": name, "started_monotonic": started, "sample_monotonic": ended,
                  "duration_ms": 1000*(ended-started), "valid": valid, "error": error,
                  "data": copy.deepcopy(data)}
        self.samples[name]["last_attempt"] = sample
        if valid:
            self.samples[name]["last_valid"] = copy.deepcopy(sample)
        # Drop overdue slots rather than issuing a burst after a slow query.
        self.next_query = max(started+self.period, ended)
        return copy.deepcopy(sample)

    def snapshot(self):
        now = self.clock()
        channels = copy.deepcopy(self.samples)
        for state in channels.values():
            good = state["last_valid"]
            state["age_seconds"] = None if good is None else max(0.,now-good["sample_monotonic"])
        return {"enabled": self.driver is not None,
                "query_rate_hz": 1./self.period, "channels": channels}


class ActuatorIO:
    """Commit ACKs, or explicitly labeled one-way writes; latch send failures.

    A latched failure blocks further motion writes. Feedback can still be read.
    No automatic retry, fallback motion, neutral motion, or emergency-stop claim.
    """
    def __init__(self, planner, sender=None, feedback_rate=10., clock=time.monotonic,
                 feedback_enabled=True):
        self.planner, self.sender, self.clock = planner, sender, clock
        self.feedback_enabled = feedback_enabled
        self.feedback = FeedbackPoller(getattr(sender,"hand",None) if feedback_enabled else None,
                                       feedback_rate, clock)
        self.fault = None
        self.sequence = 0

    def dispatch(self, proposal):
        if proposal["generation"] != self.planner.generation:
            raise ValueError("stale actuator proposal")
        started = self.clock()
        packet = prepare_packet(proposal)
        result = {"sequence": self.sequence, "attempted": False, "write_acknowledged": False,
                  "host_write_completed": False,
                  "transport_mode": getattr(self.sender,"control_transport","legacy"),
                  "hardware_enabled": self.sender is not None, "packet": packet,
                  "history_committed": False, "started_monotonic": started if self.sender is not None else None}
        self.sequence += 1
        if self.sender is None:
            self.planner.commit_preview(proposal)
            result.update(status="SIMULATED_NOT_SENT", history_committed=True)
        elif self.fault is not None:
            result.update(status="BLOCKED_AFTER_FAILURE", fault=copy.deepcopy(self.fault))
        elif packet is None:
            result.update(status="CONFIGURATION_INCOMPLETE")
        else:
            result["attempted"] = True
            try:
                succeeded = self.sender.send(packet) is True
                error = None if succeeded else "DRIVER_RETURNED_FALSE"
            except Exception as exc:
                succeeded, error = False, f"{type(exc).__name__}: {exc}"
            result["driver_result"] = copy.deepcopy(getattr(self.sender.hand,"last_move_result",None))
            if isinstance(result["driver_result"],dict):
                result["host_write_completed"] = result["driver_result"].get("host_write_completed",False)
            if succeeded:
                driver_result = result["driver_result"]
                if isinstance(driver_result,dict) and driver_result.get("status") == "HOST_WRITTEN_UNCONFIRMED":
                    self.planner.commit_transmitted(proposal)
                    result.update(status="HOST_WRITTEN_UNCONFIRMED", host_write_completed=True,
                                  history_committed=True, history_reference="last_host_written_target")
                else:
                    self.planner.commit_acknowledged(proposal)
                    result.update(status="WRITE_ACKNOWLEDGED", write_acknowledged=True, host_write_completed=True,
                                  history_committed=True, history_reference="last_bridge_confirmed_target")
            else:
                self.fault = {"reason": "SEND_FAILED_EXECUTION_UNCERTAIN", "error": error,
                              "since_monotonic": self.clock()}
                result.update(status="SEND_FAILED_EXECUTION_UNCERTAIN", fault=copy.deepcopy(self.fault))
        result["duration_ms"] = 1000*(self.clock()-started) if self.sender is not None else 0.
        return result

    def cycle(self, mapping, solver):
        proposal = self.planner.propose(mapping,solver)
        if self.sender is not None:
            proposal.update(mode="hardware_control", hardware_enabled=True)
        execution = self.dispatch(proposal)
        self.feedback.driver = getattr(self.sender,"hand",None) if self.feedback_enabled else None
        if self.feedback_enabled:
            self.feedback.poll()
        # Valid finger faults block subsequent motion commands, not readback.
        if self.sender is not None and self.fault is None:
            for name,state in self.feedback.samples.items():
                good = state["last_valid"]
                if name in FINGERS and good and good["data"].get("error_flags",0):
                    self.fault = {"reason":"ACTUATOR_FAULT", "channel":name,
                                  "since_monotonic":self.clock()}
                    break
        execution["latched_fault"] = copy.deepcopy(self.fault)
        return proposal, execution, self.feedback.snapshot()

    def reconcile_from_feedback(self):
        """Explicit recovery only: require all nine valid, post-fault readbacks.

        This updates jump-guard references; it sends no command and does not
        perform startup motion. There is deliberately no automatic caller.
        """
        if self.sender is None or self.fault is None:
            raise ValueError("reconciliation requires an active I/O fault")
        positions = {}
        now = self.clock()
        max_age = 2*len(CHANNELS)*self.feedback.period
        if self.planner.config.missing():
            raise ValueError("complete motor configuration is required")
        for name,state in self.feedback.samples.items():
            sample = state["last_valid"]
            if (sample is None or state["last_attempt"] != sample or
                    sample["started_monotonic"] <= self.fault["since_monotonic"] or
                    now-sample["sample_monotonic"] > max_age or
                    (name in FINGERS and sample["data"].get("error_flags",0))):
                raise ValueError(f"{name}: valid post-fault feedback required")
            position = sample["data"]["current_position"]
            spec = self.planner.config.actuators[name]
            if not spec["min"] <= position <= spec["max"]:
                raise ValueError(f"{name}: feedback outside configured limits")
            positions[name] = position
        self.planner.previous = positions
        self.planner.generation += 1
        self.fault = None
