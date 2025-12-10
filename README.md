# Distributed-Robot-MetaStability-Webots

## Purpose

This Webots simulation demonstrates and validates **substrate-timing-independence** for meta-state stability in distributed multi-robot swarm systems. The simulation tests how a swarm of TurtleBot3 robots can maintain consistent collective behavior (meta-states) despite variations in communication delays and timing uncertainties.

### Key Features:
- **Distributed Consensus Protocol**: Robots use a priority-based consensus mechanism to resolve simultaneous object detection conflicts without centralized coordination
- **State Transitions**: The system tracks and records state transitions (`random_walk` → `waiting_replies` → `consensus` → `path_finding` → `path_following` → `arrived`) 
- **Configurable Communication Delays**: Each robot can be assigned different communication delays (via `send_delay` parameter) to test timing-independence and robustness to asynchronous communication
- **Automated Data Collection**: Simulation runs are automatically logged with transition timestamps, delays, and contextual notes for analyzing meta-stability properties
- **Conflict Resolution**: Priority-based allegiance switching ensures deterministic consensus even when multiple robots simultaneously detect objects

The simulation supports multiple scenarios (`1-detect.wbt`, `2-detect.wbt`, `3-detect.wbt`) to test single vs. simultaneous detection events, demonstrating how the meta-state (collective swarm behavior) remains stable regardless of substrate-level timing variations.


## Quick paper-submission setup
1. `cd /Users/game/Github-Work/Distributed-Robot-MetaStability-Webots/simulation-probing`.
2. Create/activate an environment (e.g., `uv venv && source .venv/bin/activate` or your preferred Python workflow), then install the dependencies bundled in `pyproject.toml`/`uv.lock` via `uv sync` (aka “install everything in the PyTorrent”).
3. Make sure Webots is installed, its binaries are on your `PATH`, and `WEBOTS_HOME` points to your install so controllers can resolve runtime assets.
4. Launch Webots, load a world from `simulation-probing/main/worlds/` (start with `1-detect.wbt` for a single detected robot, then `2-detect.wbt` and `3-detect.wbt` for simultaneous detection), and run the simulation; the cluster controller under `main/controllers/swarm.py`.

You can now capture the console log outputs requested for the “Substrate-Timing-Independence for Meta-State Stability of Distributed Multi-Robot Clusters” submission.




# Confiurgation 
Adjustment of the communication delays for the testing of the meta-state stability `simulation-probing/main/controllers/swarm/swarmtools/communication/communicator.py: 56`
``` python
        # self.send_delay = 0.0
        if self.name[-1] == "1":
            self.send_delay = 0.1 # seconds
        elif self.name[-1] == "2":
            self.send_delay = 0.1
        elif self.name[-1] == "3":
            self.send_delay = 0.1

```