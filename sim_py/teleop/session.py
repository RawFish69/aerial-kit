"""GUI wiring for the interactive quadrotor teleop session."""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

from aerial_kit.types import SimState

from ..core.config import NormalizedSimConfig
from ..core.registry import create_backend, register_builtin_components
from .camera import CameraController
from .commands import FixedWingTeleopTuning, TeleopTuning
from .engine import TeleopEngine
from .input_state import KeyboardState, control_help_lines
from .loop import FixedStepScheduler
from .model import QuadGeometry, WingGeometry
from .renderer import HudInfo, TeleopRenderer
from .telemetry import TelemetryRecorder
from .world import TeleopWorld, build_world

logger = logging.getLogger(__name__)

#: Matplotlib keybindings that would otherwise steal teleop keys.
CONFLICTING_KEYMAPS = (
    "keymap.back",
    "keymap.copy",
    "keymap.forward",
    "keymap.fullscreen",
    "keymap.grid",
    "keymap.grid_minor",
    "keymap.help",
    "keymap.home",
    "keymap.pan",
    "keymap.quit",
    "keymap.quit_all",
    "keymap.save",
    "keymap.xscale",
    "keymap.yscale",
    "keymap.zoom",
)

FALLBACK_NON_INTERACTIVE = frozenset({"agg", "cairo", "pdf", "pgf", "ps", "svg", "template"})

#: Floor for the adaptive timer interval. Toolkits queue the next tick after
#: the redraw completes, so shrinking past this point cannot buy frames and
#: only turns the session into a busy loop.
MIN_TIMER_INTERVAL_MS = 10
#: How often the adaptive interval is revisited, in frames.
RETUNE_EVERY_FRAMES = 20

#: Sessions are kept alive here for the lifetime of their window. Without a
#: strong reference the session (and therefore the GUI timer holding the tick
#: callback) can be garbage collected, which is what makes a teleop window
#: silently sit there doing nothing.
_LIVE_SESSIONS: set["TeleopSession"] = set()


class TeleopBackendError(RuntimeError):
    """The active Matplotlib backend cannot open an interactive window."""


def ensure_interactive_backend() -> str:
    """Return the active Matplotlib backend, or raise an actionable error."""
    import matplotlib

    name = matplotlib.get_backend()
    key = str(name).lower()
    if key.startswith("module://"):
        return str(name)

    non_interactive = FALLBACK_NON_INTERACTIVE
    try:
        from matplotlib.backends.registry import BackendFilter, backend_registry

        non_interactive = frozenset(
            b.lower() for b in backend_registry.list_builtin(BackendFilter.NON_INTERACTIVE)
        )
    except Exception:  # pragma: no cover - older Matplotlib
        pass

    if key in non_interactive:
        raise TeleopBackendError(
            f"Teleop needs an interactive Matplotlib backend, but the active backend is "
            f"'{name}', which can only write files.\n"
            "Fix one of the following and retry:\n"
            "  * unset MPLBACKEND (currently forcing the backend), or set MPLBACKEND=TkAgg\n"
            "  * remove any matplotlib.use('Agg') call that runs before teleop starts\n"
            "  * install a GUI toolkit: python -m pip install pyqt6   (or install Tk support)\n"
            "  * on Linux over SSH, export DISPLAY or run with a local session\n"
            "Non-interactive runs can still use: aerial-kit-sim --no-show --save out.png"
        )
    return str(name)


def disable_conflicting_keymaps() -> dict[str, list[str]]:
    import matplotlib

    saved: dict[str, list[str]] = {}
    for key in CONFLICTING_KEYMAPS:
        if key not in matplotlib.rcParams:
            continue
        saved[key] = list(matplotlib.rcParams[key])
        matplotlib.rcParams[key] = []
    return saved


def restore_keymaps(saved: dict[str, list[str]]) -> None:
    import matplotlib

    for key, value in saved.items():
        try:
            matplotlib.rcParams[key] = value
        except Exception:  # pragma: no cover - defensive
            pass


def attach_focus_tracking(fig: Any, keyboard: KeyboardState) -> bool:
    """Clear held keys when the window loses focus. Returns True if wired up."""
    canvas = fig.canvas

    get_tk_widget = getattr(canvas, "get_tk_widget", None)
    if callable(get_tk_widget):
        try:
            widget = get_tk_widget()
        except Exception:  # pragma: no cover - defensive
            widget = None
        if widget is not None:
            # TkAgg routes keyboard events to the canvas widget, so the canvas
            # losing focus is exactly the condition under which key releases
            # would stop arriving. Treat it as lost input unconditionally --
            # failing towards "commands neutral" is the safe direction.
            widget.bind("<FocusIn>", lambda _event: keyboard.set_focused(True), add="+")
            widget.bind("<FocusOut>", lambda _event: keyboard.set_focused(False), add="+")
            try:
                # Hand keyboard focus back to the canvas when the window is
                # re-activated, so returning from another app restores control
                # without needing a click first.
                widget.winfo_toplevel().bind(
                    "<FocusIn>", lambda _event: widget.focus_set(), add="+"
                )
            except Exception:  # pragma: no cover - defensive
                pass
            return True

    try:  # Qt backends
        from matplotlib.backends.qt_compat import QtWidgets

        app = QtWidgets.QApplication.instance()
        if app is not None:
            def _on_focus_changed(_old: Any, new: Any) -> None:
                keyboard.set_focused(new is not None)

            app.focusChanged.connect(_on_focus_changed)
            return True
    except Exception:
        pass

    logger.info(
        "Window focus tracking is unavailable on this backend; use X to neutralize controls."
    )
    return False


class TeleopSession:
    """Owns the GUI timer, the engine and the renderer for one flight."""

    def __init__(
        self,
        *,
        world: TeleopWorld,
        backend_name: str,
        engine: TeleopEngine,
        renderer: TeleopRenderer,
        camera: CameraController,
        scheduler: FixedStepScheduler,
        sim_time_limit: float = 0.0,
    ) -> None:
        self.world = world
        self.backend_name = backend_name
        self.engine = engine
        self.renderer = renderer
        self.camera = camera
        self.scheduler = scheduler
        self.sim_time_limit = float(sim_time_limit)

        self.keyboard = engine.keyboard
        self.fig = renderer.fig
        # Matplotlib's own shortcuts collide with the flight controls -- 'q'
        # closes the window, 's' opens a save dialog, 'p' pans, and 'c'/'h'/'f'
        # move the view. Suppressing them here (rather than in the launcher)
        # keeps every entry point, including build_session, flyable.
        self._saved_keymaps = disable_conflicting_keymaps()
        self._closed = False
        self._window_destroyed = False
        self._seen_neutralize = engine.keyboard.neutralize_requests
        self._seen_camera_toggles = engine.keyboard.camera_toggle_requests
        self._seen_help_toggles = engine.keyboard.help_toggle_requests
        self._seen_zoom = engine.keyboard.zoom_requests
        self._neutral_frames_left = 0
        self._help_visible = True

        self.timer = self.fig.canvas.new_timer(interval=scheduler.interval_ms)
        self.timer.add_callback(self.tick)

        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)
        self.fig.canvas.mpl_connect("key_release_event", self._on_key_release)
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.focus_tracking = attach_focus_tracking(self.fig, self.keyboard)

    def _on_key_press(self, event: Any) -> None:
        self.keyboard.press(getattr(event, "key", None))

    def _on_key_release(self, event: Any) -> None:
        self.keyboard.release(getattr(event, "key", None))

    def _on_close(self, _event: Any) -> None:
        # The toolkit is already tearing the window down, so stop() must not
        # try to close it again -- doing so raises inside the Tk callback.
        self._window_destroyed = True
        self.stop()

    def _drain_view_requests(self) -> None:
        toggles = self.keyboard.camera_toggle_requests - self._seen_camera_toggles
        if toggles:
            self._seen_camera_toggles = self.keyboard.camera_toggle_requests
            for _ in range(toggles):
                self.camera.toggle_mode()
        help_toggles = self.keyboard.help_toggle_requests - self._seen_help_toggles
        if help_toggles:
            self._seen_help_toggles = self.keyboard.help_toggle_requests
            if help_toggles % 2:
                self._help_visible = not self._help_visible
                self.renderer.set_help_visible(self._help_visible)
        zoom = self.keyboard.zoom_requests - self._seen_zoom
        if zoom:
            self._seen_zoom = self.keyboard.zoom_requests
            self.camera.zoom(zoom)
        if self.keyboard.neutralize_requests != self._seen_neutralize:
            self._seen_neutralize = self.keyboard.neutralize_requests
            # Keep the HUD banner up long enough for a human to read it.
            self._neutral_frames_left = 30

    def tick(self) -> None:
        if self._closed:
            return
        if not self.keyboard.running:
            self.stop()
            return

        self._drain_view_requests()

        if self.keyboard.paused:
            self.scheduler.pause_drift()
            self.scheduler.frames += 1
            self.engine.refresh_idle_command()
        else:
            self.engine.advance(self.scheduler.tick())

        if self._neutral_frames_left > 0:
            self._neutral_frames_left -= 1

        self.renderer.update(
            self.engine.state,
            self.engine.command,
            self.engine.telemetry.trail_array(),
            self.hud_info(),
        )
        self.fig.canvas.draw_idle()
        self.retune_timer()

        if self.sim_time_limit > 0.0 and float(self.engine.state.t) >= self.sim_time_limit:
            logger.info("Teleop reached its %.1fs time limit; closing.", self.sim_time_limit)
            self.stop()

    def retune_timer(self) -> None:
        """Nudge the timer interval so the achieved frame rate meets the target.

        GUI toolkits schedule the next timer callback only after the current
        redraw finishes, so a nominal 33 ms interval plus a 27 ms draw settles
        near 17 fps instead of 30. Shrinking the interval by the observed
        shortfall recovers the target without pinning a core.
        """
        if self.scheduler.frames % RETUNE_EVERY_FRAMES:
            return
        measured = self.scheduler.frame_rate
        if measured <= 0.0:
            return
        target = self.scheduler.target_fps
        nominal = self.scheduler.interval_ms
        interval = int(self.timer.interval)
        if measured < target * 0.92 and interval > MIN_TIMER_INTERVAL_MS:
            self.timer.interval = max(MIN_TIMER_INTERVAL_MS, int(interval * 0.7))
        elif measured > target * 1.08 and interval < nominal:
            self.timer.interval = min(nominal, int(interval * 1.2) + 1)

    def hud_info(self) -> HudInfo:
        return HudInfo(
            frames=self.scheduler.frames,
            frame_rate=self.scheduler.frame_rate,
            real_time_factor=1.0 if self.keyboard.paused else self.scheduler.real_time_factor,
            sim_steps=self.engine.steps_taken,
            backend_name=self.backend_name,
            paused=self.keyboard.paused,
            focused=self.keyboard.focused,
            colliding=self.engine.colliding,
            crashed=self.engine.crashed,
            camera=self.camera.describe(),
            neutralized=self._neutral_frames_left > 0,
            motor_thrust=self.engine.motor_thrust_fractions(),
        )

    def start(self) -> None:
        _LIVE_SESSIONS.add(self)
        self.scheduler.start()
        self.renderer.update(
            self.engine.state,
            self.engine.command,
            self.engine.telemetry.trail_array(),
            self.hud_info(),
        )
        self.timer.start()

    def stop(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.keyboard.running = False
        try:
            self.timer.stop()
        except Exception:  # pragma: no cover - backend dependent
            pass
        restore_keymaps(self._saved_keymaps)
        self._saved_keymaps = {}
        _LIVE_SESSIONS.discard(self)
        if self._window_destroyed:
            return
        import matplotlib.pyplot as plt

        if plt.fignum_exists(self.fig.number):
            try:
                plt.close(self.fig)
            except Exception:  # pragma: no cover - toolkit teardown races
                logger.debug("Ignoring error while closing the teleop window", exc_info=True)


def build_session(
    cfg_norm: NormalizedSimConfig,
    *,
    target_fps: float | None = None,
) -> TeleopSession:
    """Assemble a ready-to-start teleop session from a normalized config."""
    if cfg_norm.seed is not None:
        np.random.seed(cfg_norm.seed)

    ctrl_cfg = dict(cfg_norm.controller_cfg)
    vis_cfg = dict(cfg_norm.visual_cfg)
    teleop_cfg = dict(ctrl_cfg.get("teleop", {}) or {})

    world = build_world(cfg_norm)

    register_builtin_components()
    backend_name = str(cfg_norm.backend_name).lower()
    is_fixed_wing = backend_name == "fixedwing"
    if backend_name not in {"multirotor", "rotorpy", "fixedwing"}:
        logger.warning(
            "Teleop expects a multirotor- or fixed-wing-capable backend; '%s' may "
            "ignore attitude commands.",
            backend_name,
        )
    backend = create_backend(backend_name)

    if is_fixed_wing:
        # A fixed wing has no hover: starting it at rest means zero airspeed,
        # zero lift, and an immediate stall/fall. Seed velocity and attitude
        # from initial_state config the same way aerial_kit.sim.api.run_simulation
        # does for the autonomous L1/TECS flight, so --teleop and the
        # autonomous run launch from the same physical state.
        initial_cfg = dict(cfg_norm.initial_state_cfg)
        initial_velocity = np.asarray(
            initial_cfg.get("velocity_mps", [12.0, 0.0, 0.0]), dtype=float
        ).reshape(3)
        if "attitude_quat_wxyz" in initial_cfg:
            initial_attitude = np.asarray(initial_cfg["attitude_quat_wxyz"], dtype=float)
        else:
            from aerial_kit.dynamics.fixed_wing import level_attitude_quat

            heading_rad = np.radians(float(initial_cfg.get("heading_deg", 0.0)))
            initial_attitude = level_attitude_quat(heading_rad)
        initial_body_rates = np.asarray(
            initial_cfg.get("body_rates_rps", [0.0, 0.0, 0.0]), dtype=float
        ).reshape(3)
        initial_state = SimState(
            position=world.start_position.copy(),
            velocity=initial_velocity,
            t=0.0,
            attitude_quat=initial_attitude,
            body_rates=initial_body_rates,
        )
    else:
        initial_state = SimState(
            position=world.start_position.copy(), velocity=np.zeros(3, dtype=float), t=0.0
        )

    backend.reset(
        initial_state=initial_state,
        world={
            "space_dim": world.space_dim.copy(),
            "max_z_allowed": world.max_z_allowed,
            "terrain": world.terrain,
            "terrain_clearance": world.terrain_clearance,
        },
        cfg={"controller": ctrl_cfg, "simulation": cfg_norm.simulation_cfg},
    )

    # Physics runs at a fixed step independent of the render rate; a large
    # configured dt would make attitude tracking coarse and jittery.
    dt = min(float(cfg_norm.dt) if cfg_norm.dt > 0.0 else 0.01, 0.02)
    fps = float(target_fps if target_fps is not None else teleop_cfg.get("target_fps", 30.0))
    scheduler = FixedStepScheduler(dt=dt, target_fps=fps)

    tuning: TeleopTuning | FixedWingTeleopTuning = (
        FixedWingTeleopTuning.from_config(ctrl_cfg)
        if is_fixed_wing
        else TeleopTuning.from_config(ctrl_cfg)
    )
    engine = TeleopEngine(
        backend=backend,
        world=world,
        tuning=tuning,
        dt=dt,
        keyboard=KeyboardState(),
        telemetry=TelemetryRecorder(stride=max(1, int(round(0.05 / dt)))),
    )

    # A fixed wing covers ground much faster than a hovering quad and never
    # loiters, so it needs a wider follow cube by default -- the quad's
    # close-chase radius would put it outside the frame within a second or
    # two of flight. Elevation is kept low for both so the live view reads as
    # a camera sitting just behind and above the vehicle (a third-person
    # chase cam) rather than an overhead survey shot; the wing's is a little
    # higher than the quad's so a banked turn doesn't foreshorten flat.
    default_radius = 35.0 if is_fixed_wing else 12.0
    default_elevation = 13.0 if is_fixed_wing else 9.0
    camera = CameraController(
        world_bounds=world.camera_bounds,
        radius=float(vis_cfg.get("teleop_view_radius_m", default_radius)),
        min_radius=float(vis_cfg.get("teleop_view_radius_min_m", 5.0)),
        max_radius=float(max(world.space_dim[:2].max(), 60.0)),
        elevation_deg=float(vis_cfg.get("teleop_view_elevation_deg", default_elevation)),
        azimuth_deg=float(vis_cfg.get("teleop_view_azimuth_deg", -60.0)),
        world_elevation_deg=float(vis_cfg.get("teleop_world_elevation_deg", 34.0)),
        world_azimuth_deg=(
            float(vis_cfg["teleop_world_azimuth_deg"])
            if vis_cfg.get("teleop_world_azimuth_deg") is not None
            else None
        ),
        canvas_fill=float(vis_cfg.get("teleop_view_zoom", 1.45)),
    )
    geometry: QuadGeometry | WingGeometry
    if is_fixed_wing:
        geometry = WingGeometry(
            fuselage_length=float(vis_cfg.get("teleop_wing_fuselage_m", 6.8)),
            wingspan=float(vis_cfg.get("teleop_wing_span_m", 10.6)),
            chord=float(vis_cfg.get("teleop_wing_chord_m", 1.45)),
        )
    else:
        geometry = QuadGeometry(
            arm_length=float(vis_cfg.get("teleop_quad_arm_m", 0.9)),
            scale=float(vis_cfg.get("teleop_model_scale", 3.0)),
        )
    renderer = TeleopRenderer(
        world=world,
        camera=camera,
        geometry=geometry,
        backend_name=backend_name,
        visual_cfg=vis_cfg,
    )

    return TeleopSession(
        world=world,
        backend_name=backend_name,
        engine=engine,
        renderer=renderer,
        camera=camera,
        scheduler=scheduler,
        sim_time_limit=float(cfg_norm.sim_time),
    )


def run_teleop_session(cfg_norm: NormalizedSimConfig) -> None:
    """Open the interactive teleop window and fly until the user exits."""
    ensure_interactive_backend()
    import matplotlib.pyplot as plt

    session: TeleopSession | None = None
    try:
        session = build_session(cfg_norm)
        logger.info(
            "Teleop ready: backend=%s dt=%.3fs world=%s",
            session.backend_name,
            session.engine.dt,
            np.round(session.world.space_dim, 1).tolist(),
        )
        for line in control_help_lines(is_fixed_wing=session.backend_name == "fixedwing"):
            logger.info("%s", line)
        session.start()
        plt.show()
    finally:
        if session is not None:
            # Also restores the Matplotlib keymaps the session suppressed.
            session.stop()


def run_teleop_cli(cfg_norm: NormalizedSimConfig) -> None:
    """Launcher wrapper: report an unusable backend without a traceback."""
    try:
        run_teleop_session(cfg_norm)
    except TeleopBackendError as exc:
        raise SystemExit(f"aerial-kit teleop: {exc}") from None


__all__ = [
    "CONFLICTING_KEYMAPS",
    "TeleopBackendError",
    "TeleopSession",
    "attach_focus_tracking",
    "build_session",
    "disable_conflicting_keymaps",
    "ensure_interactive_backend",
    "restore_keymaps",
    "run_teleop_cli",
    "run_teleop_session",
]
