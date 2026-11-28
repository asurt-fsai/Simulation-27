"""ROS-independent CARLA LiDAR packet converter and bounded scan accumulator."""

from collections import deque
import numpy as np


def decode_carla_lidar(raw_data):
    """CARLA ray_cast float32 x,y,z,intensity -> ROS FLU x,y,z,intensity.

    CARLA uses x-forward, y-right, z-up; ROS base frames use y-left. Do not
    rotate into odom or use actor ground truth. The cloud stays sensor-local.
    """
    source = np.frombuffer(raw_data, dtype="<f4")
    if source.size % 4:
        raise ValueError("CARLA LiDAR buffer is not a multiple of four floats")
    xyz_i = source.reshape(-1, 4).copy()
    xyz_i[:, 1] *= -1.0
    return xyz_i


class BankFrameAssembler:
    """Join a frame's banks in order, allowing a bounded callback delay."""

    def __init__(self, bank_count, max_lag_frames=4):
        self.bank_count = bank_count
        self.max_lag_frames = max_lag_frames
        self.pending = {}
        self.last_frame = -1
        self.dropped_frames = 0
        self.late_banks = 0

    def add(self, frame, timestamp, bank, points):
        if frame <= self.last_frame:
            self.late_banks += 1
            return []

        _, parts = self.pending.setdefault(frame, (timestamp, {}))
        parts.setdefault(bank, points)
        newest = max(self.pending)
        ready = []
        while self.pending:
            oldest = min(self.pending)
            stamp, parts = self.pending[oldest]
            if len(parts) == self.bank_count:
                ready.append((stamp, np.concatenate([parts[i] for i in range(self.bank_count)])))
            elif newest - oldest >= self.max_lag_frames:
                self.dropped_frames += 1
            else:
                break
            del self.pending[oldest]
            self.last_frame = oldest
        return ready


class RevolutionAccumulator:
    """Group simulated CARLA tick segments over approximately one rotation.

    A ray_cast actor emits a segment each simulation tick. Publishing each
    segment as a complete 360-degree LiDAR revolution causes false negatives.
    A fixed simulation-time boundary is advanced after each sweep. Resetting
    the boundary from the first packet after every publication wastes one
    CARLA tick per sweep (10 Hz becomes about 6 Hz at a 0.033-s world tick).
    Preserve the scan phase and the latest measurement stamp across sweeps.
    """

    def __init__(self, rotation_hz=10.0, max_packets=12):
        if not 0.1 <= rotation_hz <= 30:
            raise ValueError("invalid rotation_hz")
        if not 2 <= max_packets <= 100:
            raise ValueError("invalid max_packets")
        self.period = 1.0 / rotation_hz
        self.packets = deque(maxlen=max_packets)
        self.start_stamp = None
        self.next_boundary = None
        self.last_stamp = None
        self.dropped_packets = 0
        self.out_of_order = 0

    def add(self, timestamp, points):
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError("invalid LiDAR timestamp")
        if self.last_stamp is not None and timestamp <= self.last_stamp:
            self.out_of_order += 1
            return None
        if not isinstance(points, np.ndarray) or points.ndim != 2 or points.shape[1] != 4:
            raise ValueError("expected N x 4 CARLA points")
        if self.start_stamp is None:
            self.start_stamp = timestamp
            self.next_boundary = timestamp + self.period
        if self.last_stamp is not None and timestamp - self.last_stamp > self.period + 1e-7:
            # Missing a sweep's worth of source sectors cannot yield a
            # trustworthy 360-degree scan. Resume with a fresh full window.
            self.dropped_packets += len(self.packets)
            self.packets.clear()
            self.next_boundary = timestamp + self.period
        if len(self.packets) == self.packets.maxlen:
            self.dropped_packets += 1
        self.packets.append(np.ascontiguousarray(points, dtype=np.float32))
        self.last_stamp = timestamp
        if timestamp + 1e-7 < self.next_boundary:
            return None
        merged = np.concatenate(self.packets) if self.packets else np.empty((0, 4), dtype=np.float32)
        self.packets.clear()
        # Retain the established rotation phase between publications. When
        # ticks straddle the deadline, advance to the next future boundary.
        self.next_boundary += self.period * max(1, int((timestamp - self.next_boundary) / self.period) + 1)
        return timestamp, merged
