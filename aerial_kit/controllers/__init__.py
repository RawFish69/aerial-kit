"""Controller implementations (numpy/scipy-only, no ROS/matplotlib)."""

from .basic import LQRController, MPCController, PIDController
from .cascade import CascadeController, CascadeGains
from .fixed_wing import AttitudeGains, FixedWingL1TECSController, body_axis_pitch_bank
from .geometric import FlatReference, GeometricController, GeometricGains
from .minimum_snap import minimum_snap_trajectory
from .mpc import ConstrainedMPC, MPCSolution
from .nmpc import NMPCController, NMPCGains, NMPCSolution, QuadrotorNMPC
from .mppi import MPPI, MPPISolution, SphereObstacle
from .predictive import ConstrainedMPCController, WarmMPPIController
from .qp import BoxQP
from .reference import HorizonReference, PathReference, constant_reference
from .position import (
    lqr_gain_double_integrator,
    lqr_position_control,
    mpc_position_control,
    mppi_position_control,
    pid_position_control,
)

__all__ = [
    "PIDController",
    "LQRController",
    "MPCController",
    "CascadeController",
    "CascadeGains",
    "FixedWingL1TECSController",
    "AttitudeGains",
    "body_axis_pitch_bank",
    "pid_position_control",
    "lqr_gain_double_integrator",
    "lqr_position_control",
    "mpc_position_control",
    "mppi_position_control",
    "minimum_snap_trajectory",
    "ConstrainedMPC",
    "MPCSolution",
    "FlatReference",
    "GeometricController",
    "GeometricGains",
    "NMPCController",
    "NMPCGains",
    "NMPCSolution",
    "QuadrotorNMPC",
    "ConstrainedMPCController",
    "MPPI",
    "MPPISolution",
    "SphereObstacle",
    "WarmMPPIController",
    "BoxQP",
    "HorizonReference",
    "PathReference",
    "constant_reference",
]
