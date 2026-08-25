# AGENTS.md — field_control

## 1. Purpose

`field_control` is embedded control software for autonomous row navigation.

The software combines:

* OAK-D SR camera input;
* BNO086 IMU data;
* HSV-based row/target detection;
* heading estimation and heading reference;
* state-machine based navigation;
* differential drive control;
* odometry;
* row-end detection and turning;
* diagnostics;
* web-based monitoring;
* livestreams;
* CAN motor control.

The implementation must prioritize:

1. deterministic behavior;
2. fail-closed safety;
3. testability;
4. clear state transitions;
5. explicit timing and units;
6. preservation of verified existing behavior;
7. maintainability on Raspberry Pi hardware.

---

## 2. Authoritative requirements

The numbered `field_control` specification, currently consisting of items
1–41, is the authoritative functional specification for the project.

Implement against that specification.

If the specification is later stored in a repository document such as:

`docs/requirements.md`

that repository document becomes the preferred authoritative reference.

Do not silently reinterpret, simplify, omit, or expand requirements.

If source code and requirements disagree, treat the requirements as
authoritative unless the user explicitly states that verified existing behavior
must be preserved.

Verified behavior explicitly referenced by the requirements must be preserved.

In particular, heading filtering must follow the already verified
`get_heading` implementation referenced by the project specification rather
than introducing a new filtering method unless explicitly requested.

---

## 3. Requirement progression

Treat unfinished applicable items in the numbered specification as a work queue.

Completing one requirement does NOT mean the delegated task is complete when
additional requested and unblocked requirements remain.

When one implementation item is complete:

1. test it;
2. verify that existing behavior remains intact;
3. determine the next natural dependency or unfinished requirement;
4. continue automatically.

Do not repeatedly ask the user which specification item to implement next when
the next step can reasonably be determined from:

* the numbered specification;
* this `AGENTS.md`;
* repository documentation;
* existing source code;
* tests;
* established project architecture.

Dependency order may override numerical requirement order when necessary.

Prefer coherent end-to-end functionality over isolated unused components.

---

## 4. Autonomous execution protocol

This project is intended to be developed autonomously.

### 4.1 Keep working

Do not stop an execution turn merely to report progress.

A progress update is NOT a completion condition.

After every progress update, immediately continue with the next:

* repository inspection;
* tool call;
* code change;
* test;
* diagnostic check;
* verification step;

required to advance the delegated task.

Do not require the user to repeatedly write:

* `continue`;
* `proceed`;
* `start`;
* `go on`;
* `sätt igång`;
* or similar.

When multiple reasonable tasks remain, choose the next natural one yourself.

Default behavior:

**KEEP WORKING.**

---

## 5. Valid reasons to stop

Return a final response only when at least one of the following applies:

1. The requested task or explicitly defined milestone is complete and tested.

2. A genuine external blocker prevents all remaining useful work.

3. Further progress requires unavailable physical hardware and no meaningful
   implementation, simulation, mocking, static verification, testing or other
   software work remains.

4. User input is genuinely required because no safe and reasonable choice can
   be derived from:

   * requirements;
   * repository documentation;
   * existing implementation;
   * tests;
   * verified referenced code;
   * established project conventions.

Do not stop merely because:

* one subtask is finished;
* one file has been implemented;
* baseline tests pass;
* a natural intermediate milestone has been reached;
* you have identified the next step;
* you want to report progress;
* a test initially fails;
* hardware-dependent verification is unavailable while other work remains;
* you have implemented a component but not yet integrated it.

---

## 6. Progress messages

Progress messages are allowed and useful during long work sessions.

However, a progress message must be followed by continued repository/tool work
in the SAME execution turn.

Never use a final response merely to say:

* "I have started..."
* "I have begun implementing..."
* "The baseline tests pass..."
* "Next I will..."
* "The next natural step is..."
* "I am now implementing..."
* "Runtime integration has started..."

Those statements describe progress, not completion.

A progress update must not return control to the user unless a valid stopping
condition from section 5 has been reached.

---

## 7. Baseline verification

Before making substantial changes:

1. inspect the relevant source code;
2. inspect relevant tests;
3. inspect applicable documentation;
4. run focused existing tests where practical;
5. establish the current baseline.

Passing baseline tests establishes only that the starting point is healthy.

It does NOT mean that the delegated implementation task is complete.

After baseline verification, continue directly with implementation.

---

## 8. Preserve verified behavior

Inspect existing implementations before introducing replacement logic.

Prefer reuse and integration of verified project code over reimplementation.

Do not create duplicate implementations of functionality that already exists
and is verified.

Preserve behavior outside the delegated scope unless changing it is necessary
to satisfy the specification.

Examples include:

* verified HSV processing;
* verified heading filtering;
* verified heading compensation;
* verified state-machine behavior;
* verified motor-unit conversion;
* verified direction conventions;
* verified timeout behavior.

If changing verified behavior becomes necessary, explain the reason in the final
report and add focused regression tests.

---

## 9. Architecture principles

Keep hardware acquisition, processing, state logic and physical output
separated.

Prefer explicit components for:

* camera acquisition;
* IMU acquisition;
* vision processing;
* heading processing;
* observation aggregation;
* navigation/state-machine logic;
* odometry;
* motor command generation;
* motor safety boundary;
* diagnostics;
* web service;
* application lifecycle.

The control/state-machine layer should not perform blocking hardware reads.

Sensor producers should provide latest-value data to consumers.

Slow diagnostics or web clients must not block navigation/control execution.

Avoid unbounded queues of stale camera or IMU data.

---

## 10. Time handling

Use monotonic time for:

* sensor freshness;
* control timeouts;
* state timeouts;
* watchdogs;
* leases;
* delays;
* elapsed-time calculations.

Do not use wall-clock time for safety-critical or control timeout logic.

Prefer:

`time.monotonic()`

or an equivalent monotonic source.

Wall-clock timestamps may additionally be used for logging and human-readable
diagnostics.

---

## 11. Sensor latest-value semantics

Camera and IMU acquisition should operate independently.

Each sensor source should expose at least:

* latest value/data;
* monotonic update timestamp;
* validity/status;
* relevant error state.

Consumers should process the most recent valid value.

Do not allow an unbounded backlog of old sensor data to influence current
navigation.

Sensor age must be explicitly available to runtime logic and diagnostics.

---

## 12. Camera and vision behavior

The camera source is the OAK-D SR.

Use the existing verified camera configuration where referenced by the project.

Vision processing must preserve the specified HSV-based detection behavior.

Relevant outputs should be represented explicitly, including where applicable:

* detected buds;
* detected foliage;
* turn marker;
* target position;
* filtered target position;
* normalized zones;
* target validity;
* original image;
* HSV mask or equivalent processed image.

Do not silently continue using indefinitely old vision results after camera data
becomes stale.

Camera failure or stale data must be represented explicitly.

---

## 13. IMU and heading behavior

The OAK-D SR BNO086 IMU is used for heading information.

Reuse the verified heading handling referenced by the project requirements.

Do not replace the verified heading filter with a new arbitrary filter.

Where required, heading must account for camera/IMU orientation and tilt using
the verified method.

Represent explicitly:

* current heading;
* heading validity;
* heading age;
* filtered heading;
* row heading reference;
* whether a valid heading reference currently exists.

Do not silently use indefinitely stale IMU data for navigation.

---

## 14. Row heading reference

`row_heading_reference` must represent the robot's estimated actual row
direction, not simply the most recent instantaneous heading.

Build/update it only according to the project specification.

The configurable parameter:

`heading_reference_min_distance_m`

must be respected.

Do not create a new heading-reference method that contradicts the numbered
requirements.

---

## 15. Observation model

Navigation logic should receive a coherent observation rather than performing
direct blocking reads from individual sensors.

The observation should contain the relevant current state of the world,
including as applicable:

* vision results;
* target information;
* marker information;
* heading;
* row heading reference;
* camera freshness;
* IMU freshness;
* sensor validity;
* odometry;
* elapsed distance;
* relevant timestamps or data ages.

Make invalid or unavailable data explicit.

Avoid using sentinel values whose meaning is unclear.

---

## 16. State machine

Navigation behavior must be represented through explicit states.

Use the states defined by the project specification.

State transitions must be:

* explicit;
* deterministic;
* testable;
* visible in diagnostics.

Do not hide major navigation modes inside loosely coupled boolean flags.

State transitions that depend on time must use monotonic time.

State transitions that depend on travelled distance must use odometry/distance
rather than assumed elapsed time unless explicitly specified otherwise.

Fault conditions must transition to safe behavior.

---

## 17. MANUAL and AUTO separation

MANUAL and AUTO behavior must remain clearly separated.

AUTO logic must not accidentally continue issuing commands after control has
been transferred to MANUAL or after AUTO has been stopped.

Mode transitions must be explicit.

Relevant state should be visible in diagnostics.

---

## 18. Odometry

Use the project-specified wheel geometry and motor gearing.

Do not assume identical wheel geometry if the configuration allows separate
wheel circumferences.

Use:

`wheel_track_m`

for differential-drive geometry wherever relevant.

Keep units explicit.

Prefer SI internally:

* distance: meters;
* velocity: meters/second where applicable;
* angles: documented degrees or radians;
* time: seconds.

Where motor APIs require RPM or motor-side units, convert explicitly at the
hardware boundary.

---

## 19. Motor speed and units

Motor commands must use the verified MyActuator protocol and conversion rules
defined by the project.

Do not guess:

* gear ratio;
* direction sign;
* RPM scaling;
* CAN command format;
* byte order;
* motor ID.

These must come from project configuration or verified project documentation.

User-configured RPM refers to the output shaft where specified.

Make motor-side/output-side conversions explicit and testable.

---

## 20. Differential steering

Differential steering must obey the project specification.

Do not exceed configured speed limits.

Keep direction signs explicit.

Make steering logic deterministic and unit-testable independently from physical
CAN hardware.

Separate:

1. desired navigation motion;
2. calculated left/right wheel command;
3. CAN encoding/output.

This allows control logic to be tested without motors.

---

## 21. Turning behavior

Turning behavior must follow the numbered requirements.

Use the configurable:

`turn_speed_rpm`

where specified.

Do not hard-code a duplicate turn RPM elsewhere.

Turning states, completion conditions and timeout/fault behavior must be
explicit and tested.

---

## 22. Automatic start delay

Use the configurable:

`auto_start_delay_s`

for the specified AUTO start delay.

Use monotonic elapsed time.

Do not implement the delay using blocking `sleep()` calls in the main control
loop if doing so prevents:

* watchdog handling;
* web diagnostics;
* stop commands;
* fault detection;
* lifecycle handling.

---

## 23. Configuration

Behavioral and physical tuning values must live in configuration rather than be
scattered as hard-coded constants.

The configurable parameters are defined by requirement 41 of the numbered
project specification.

That list is authoritative.

Examples include:

* `heading_reference_min_distance_m`;
* `turn_speed_rpm`;
* `auto_start_delay_s`;
* `wheel_track_m`;

and the other parameters defined by requirement 41.

Do not introduce duplicate configuration names for the same concept.

Do not silently change defaults.

Validate configuration at startup where practical.

Reject impossible or unsafe values explicitly.

---

## 24. CAN and physical hardware status

During software development, physical CAN and motors may intentionally be
disconnected.

This is expected.

Unavailable CAN or motors are NOT a blocker for work that can be:

* implemented;
* unit tested;
* integration tested with mocks/fakes;
* statically checked;
* simulated;
* inspected;
* verified without physical actuation.

If one item genuinely requires physical hardware:

1. mark only that verification as hardware-pending;
2. keep the affected output fail-closed;
3. continue every other unblocked task;
4. document the required hardware test in the final report.

Do not stop the entire implementation merely because CAN hardware is absent.

---

## 25. Motor-output safety boundary

Physical motor output is safety-relevant.

Motor/CAN output must remain disabled and unarmed until the required safety
mechanisms have been implemented and verified to the extent possible without
physical hardware.

This includes applicable:

* control lease;
* watchdog;
* command timeout;
* stale-control detection;
* communication-failure handling;
* stop behavior;
* fail-closed transitions.

Do not enable physical motor output merely to simplify development or testing.

No code path should unexpectedly arm motors as a side effect of:

* starting the web server;
* opening diagnostics;
* starting camera acquisition;
* starting the application;
* running tests.

---

## 26. Fail-closed behavior

When safety-relevant uncertainty exists, prefer stopping/zero command over
continuing on stale or invalid control information.

Examples include:

* expired control lease;
* watchdog timeout;
* invalid critical configuration;
* stale required sensor data where navigation cannot safely continue;
* internal runtime fault;
* loss of motor-control authority.

Exactly which sensor failures allow continued heading-only, vision-only,
SEARCH, or other behavior must follow the numbered project specification.

Do not add extra fail-open behavior beyond the requirements.

---

## 27. Watchdog and control lease

Watchdog and control-lease logic must be:

* monotonic-time based;
* explicit;
* independently testable;
* fail-closed;
* separated from UI rendering.

Expiry must reliably result in the specified safe motor behavior.

Once implemented, add tests for at least:

* valid lease;
* expired lease;
* missing refresh;
* delayed refresh;
* shutdown;
* relevant communication failure.

Do not arm physical output before this safety boundary is verified.

---

## 28. Web interface

The web interface is primarily an operator and diagnostic interface.

It must not bypass control safety layers.

Diagnostics should expose relevant runtime information, including as applicable:

* application status;
* current mode;
* state-machine state;
* camera status;
* camera age;
* IMU status;
* IMU age;
* heading;
* row heading reference;
* current target;
* target validity;
* marker state;
* odometry/distance;
* commanded left/right speed;
* motor-output armed/unarmed state;
* watchdog/lease status;
* active faults;
* relevant timeout state.

Keep diagnostic generation separated from core control calculations where
possible.

---

## 29. Livestreams

Provide the livestreams required by the project specification.

At minimum where requested:

* original camera stream;
* HSV/mask or equivalent processed stream.

Streaming must not block the control loop.

Slow or disconnected browser clients must not cause camera-frame backlogs or
navigation stalls.

Diagnostics/livestream failures must not unexpectedly terminate motor safety or
navigation processes unless required for safety.

---

## 30. Application lifecycle

The top-level application should explicitly own and coordinate its components.

Startup and shutdown must be deterministic.

Components should be started in a controlled order and stopped cleanly.

Background:

* threads;
* tasks;
* queues;
* sockets;
* camera resources;
* IMU resources;
* web resources;
* CAN resources;

must not be left running unintentionally after shutdown.

Handle Ctrl-C/SIGINT cleanly where practical.

Shutdown should leave motor output in the safe state.

---

## 31. Concurrency

Minimize shared mutable state.

When state is shared between acquisition and control components, use clear
thread/task-safe ownership or synchronization.

Avoid holding locks while performing:

* slow image processing;
* network I/O;
* web streaming;
* blocking hardware I/O.

Prefer latest-value snapshots over shared structures modified incrementally.

Document non-obvious concurrency assumptions.

---

## 32. Error handling

Do not silently swallow exceptions that can affect runtime correctness.

Background-worker failures must become visible through:

* status;
* logs;
* diagnostics;
* fault handling;

as appropriate.

Differentiate where practical between:

* expected unavailable hardware;
* stale sensor data;
* configuration error;
* transient sensor error;
* programming/runtime error;
* safety-critical fault.

Avoid broad `except Exception: pass` patterns.

---

## 33. Logging

Use structured and useful logging.

Log important lifecycle events such as:

* application startup;
* configuration;
* sensor connection;
* sensor failure/recovery;
* state transitions;
* mode transitions;
* fault entry;
* fault recovery where permitted;
* motor-output arm/disarm;
* watchdog/lease expiry;
* application shutdown.

Avoid flooding logs from high-frequency loops.

Rate-limit repetitive warnings where useful.

---

## 34. Testing strategy

Tests are a required part of implementation, not a separate optional phase.

Add focused automated tests whenever behavior changes.

Prioritize tests for:

* state transitions;
* timing;
* timeout behavior;
* sensor freshness;
* heading logic;
* target/zone logic;
* odometry;
* differential steering;
* unit conversions;
* direction signs;
* configuration;
* watchdog;
* control lease;
* fail-closed behavior;
* latest-value semantics;
* observation construction;
* application lifecycle;
* motor-output arming boundaries.

Tests should not require physical motors unless explicitly marked as hardware
tests.

Use mocks/fakes for hardware-independent tests.

---

## 35. Timing tests

Do not create flaky tests that depend on actual wall-clock sleeping when timing
can instead be controlled through an injected/fake clock.

Where practical, make monotonic time injectable.

Test important boundaries around timeouts, for example:

* just before expiry;
* exactly at expiry according to defined semantics;
* just after expiry.

---

## 36. Hardware-independent integration tests

Build integration coverage that can run with CAN and motors disconnected.

Where practical, provide fake/mock implementations for:

* camera source;
* IMU source;
* CAN/motor output;
* clock/time source.

This should allow state-machine and runtime behavior to be verified without
physical actuation.

Do not make routine CI/test execution depend on connected motors.

---

## 37. Test execution

For each delegated implementation:

1. run focused tests for changed functionality;
2. fix relevant failures;
3. rerun focused tests;
4. run the broader existing project test suite where appropriate;
5. run available lint/static/type/import checks relevant to the change.

If a test fails:

1. investigate;
2. identify whether the failure is:

   * caused by the change;
   * pre-existing;
   * environmental;
   * hardware-dependent;
3. fix in-scope defects where reasonably possible;
4. rerun the test;
5. continue working.

A test failure is NOT by itself a reason to stop and ask the user what to do.

---

## 38. Hardware tests

Hardware tests must be clearly distinguished from normal automated tests.

Do not accidentally run tests that can move physical motors.

Any test capable of physical actuation must require an explicit hardware-test
mode or equivalent deliberate opt-in.

When motors are disconnected, hardware-only tests may be recorded as pending.

Do not treat this as failure of the software-only milestone.

---

## 39. Code quality

Prefer:

* small cohesive modules;
* explicit interfaces;
* descriptive names;
* typed data structures where useful;
* enums for states/modes where appropriate;
* pure functions for calculations;
* dependency injection for hardware and time where practical.

Avoid:

* unnecessary abstraction;
* speculative frameworks;
* duplicate state;
* hidden global mutable state;
* unexplained magic numbers;
* hard-coded configuration;
* broad unrelated refactors.

Keep changes bounded to the task.

---

## 40. Units and naming

Make units visible in variable and configuration names where ambiguity is
possible.

Examples:

* `_m`;
* `_m_s`;
* `_s`;
* `_rpm`;
* `_deg`;
* `_rad`;
* `_px`.

Do not mix:

* motor RPM and output-shaft RPM;
* degrees and radians;
* seconds and milliseconds;
* meters and millimeters;

without explicit conversion.

Add tests for critical conversions.

---

## 41. Repository inspection before changes

Before adding a new module or major class:

1. search the repository for existing equivalent functionality;
2. inspect relevant call sites;
3. inspect tests;
4. inspect configuration;
5. reuse existing verified components when appropriate.

Do not assume functionality is missing solely because it is not in the first
file inspected.

---

## 42. Documentation

Update documentation when changes alter:

* configuration;
* startup procedure;
* runtime architecture;
* safety behavior;
* operator behavior;
* hardware assumptions;
* diagnostics;
* test procedure.

Do not duplicate the full numbered specification unnecessarily.

Keep implementation documentation consistent with the authoritative
requirements.

---

## 43. Scope control

Implement the delegated requirement, including necessary integration and tests.

Do not expand scope into unrelated cleanup or architectural rewriting.

Small supporting refactors are acceptable when needed to make the requested
change safe and testable.

If an unrelated defect is discovered:

* fix it only if it directly blocks or compromises the delegated task;
* otherwise record it for the final report and continue.

---

## 44. Reasonable assumptions

Do not interrupt implementation for low-risk choices that can reasonably be
made from project context.

When multiple reasonable implementation choices exist:

1. prefer the simplest design consistent with the requirements;
2. prefer existing project conventions;
3. prefer explicit/testable behavior;
4. make the choice;
5. continue;
6. mention important non-obvious choices in the final report.

Ask the user only when the decision materially changes required external
behavior and cannot reasonably be derived from the specification.

---

## 45. Definition of Done

Unless the delegated task defines a more specific Definition of Done, work is
complete when:

* requested functionality is implemented;
* required integrations are complete;
* configuration is wired correctly;
* relevant safety boundaries are preserved;
* focused tests have been added or updated;
* focused tests pass;
* applicable existing tests pass;
* relevant lint/type/import/static checks have been run;
* hardware-independent verification has been completed;
* hardware-only verification still required is clearly identified;
* documentation is updated when required.

Merely creating classes, files, interfaces or placeholders does not satisfy
Definition of Done.

---

## 46. Final report

Only after reaching Definition of Done, provide a concise final report.

Include:

### Changed files

List files added or modified.

### Completed work

Summarize functionality actually implemented.

### Key decisions

Mention important architectural or behavioral decisions.

### Tests and checks

List tests/checks actually run and their results.

### Hardware verification remaining

State exactly what still requires connected hardware, if anything.

### Remaining risks or blockers

List only genuine unresolved issues.

Do not use the final report to announce work that you merely intend to perform.

If useful unblocked work remains inside the delegated task:

**continue doing it instead of returning the final report.**

---

## 47. Subagents

When subagents are available, use them where they improve quality without
unnecessarily fragmenting the task.

Suitable delegation includes:

* focused implementation;
* independent code review;
* targeted test review;
* safety verification;
* investigation of isolated failures.

The main agent remains responsible for:

* understanding the complete requirement;
* coordinating delegated work;
* integrating changes;
* resolving conflicts;
* running final relevant verification;
* determining whether Definition of Done is satisfied.

A subagent completing its delegated task does NOT mean the main task is
complete.

The main agent must continue automatically after receiving a subagent result.

---

## 48. Safety-sensitive review triggers

Give additional scrutiny to changes involving:

* physical motor behavior;
* motor direction;
* RPM or unit conversion;
* CAN command encoding;
* watchdogs;
* control leases;
* timeouts;
* stale sensor behavior;
* fail-closed behavior;
* state-machine transitions;
* concurrency;
* sensor validity;
* odometry;
* wheel geometry;
* heading calculation;
* configuration affecting physical movement.

For these areas, prefer focused tests plus independent review/verification when
available.

---

## 49. Project-specific current hardware-development rule

Unless the user explicitly states otherwise during a task:

* physical CAN may be unavailable;
* physical drive motors may be disconnected.

Continue all software work that does not require physical motor actuation.

Do not repeatedly report disconnected motors as a blocker.

Keep the physical motor-output boundary unarmed until the applicable safety
integration has been implemented and verified.

When hardware later becomes available, perform only the specifically required
hardware verification and preserve the previously tested software behavior.

---
[text](../../Music)
## 50. Core operating rule

When deciding whether to return control to the user, ask:

> Is the delegated task actually complete, or is there still useful,
> unblocked work I can perform now?

If useful unblocked work remains:

**DO THE WORK. DO NOT STOP TO ASK FOR PERMISSION TO CONTINUE.**


