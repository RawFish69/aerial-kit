> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - return to home

The fixed wing's failsafe, when somebody has taught it where home is: hold a
cruise throttle, steer toward home with roll, hold the altitude it had when it
lost the link, and circle when it arrives.

And, on the same steering, a mission: a short list of places to go, flown in
order while the pilot still has the transmitter.

## The shape of it

```text
GPS fix ──> ak_nav ──> stick-shaped command ──> flight core ──> mixer ──> motors
                       (roll, pitch, throttle)   (only when the
                                                  pilot's link is gone)
```

The navigator produces the same thing the RC decoder produces, which is why the
control loop and the mixer need no idea that a navigator exists. A module that
drove the flight loop directly would have to be re-tested against every airframe;
one that produces stick positions only has to be right about where home is.

The flight core's side of the contract is small and worth stating:

- **Guidance is only used when the pilot's link is gone.** A live link always
  wins, and a returning link hands control straight back to the sticks.
- **Guidance cannot arm.** A navigator decides where to fly, never whether to be
  armed or disarmed.
- **Guidance does not fly a disarmed aircraft.** The state has to be armed (or
  already returning) for it to take over.
- **No attitude, no flying.** An invalid IMU stops the aircraft whoever is
  talking.

## The arithmetic worth staring at

The course error is the one that flies the aircraft away. Wrapping a heading
difference wrongly turns a ten degree error into a 350 degree one, and it only
shows up when the two headings straddle north - which is exactly when it matters.
So the wrap goes through `sin`/`cos`/`atan2` rather than a pair of `if`
statements: `atan2(sin(δ), cos(δ))` is the angle in -180..180 by construction and
cannot be got wrong at the boundary. That boundary is tested on both sides, at
exactly opposite, and at zero.

Distance and bearing use the equirectangular approximation - a metre or so of
error at a kilometre, shrinking with distance - which is far below what a loiter
radius of a few tens of metres cares about, and it costs a `cos` instead of a
spherical triangle.

## Home

Home is captured from the first valid fix while the aircraft is disarmed, which
is the take-off point and the only moment it can be captured without asking.
`home` on the console sets it by hand from the current fix (what a bench session
does), and `home clear` forgets it.

## Turning it on, and leaving it off

`rth_enable` defaults to **0**: a lost link stops the aircraft. That is the
behaviour to have until a return has been tested, and it is deliberately not the
behaviour a wing wants in the air either - which is why the bench procedure is:

```text
ak> set rth_enable 1
# props off, receiver off, aircraft held:
ak> gps            # home set, distance to it
ak> status         # state: returning home
```

Watch the elevons deflect toward home and the motors go to cruise, then take the
receiver back and check control returns to the sticks. Only after that is the
setting worth keeping - `save` keeps it.

## The mission

A return is something a navigator does *to* an aircraft when nobody else can. A
mission is something it does *for* a pilot who is still there: the link is up,
the sticks are connected, and the navigator is flying anyway because it was
handed the aircraft. That difference is one `if` in the flight core, and it is
the whole of the new state:

```text
ak> mission add 52.1254304 4.9876548
mission: waypoint 0 is 52.1254304, 4.9876548 - 'save' keeps it
ak> mission add 52.1254304 4.9916792
mission: waypoint 1 is 52.1254304, 4.9916792 - 'save' keeps it
ak> set mission_channel 7
mission_channel = 7
# transmitter: channel 7 high
ak> mission
mission:   2 waypoints, flying
  [0] 52.1254304, 4.9876548
  [1] 52.1254304, 4.9916792  <- flying here
  1 reached, holding 120000 mm
```

Four things are deliberate:

- **Waypoints are parameters.** `wp0_lat` .. `wp3_lon` and `wp_count` are in the
  same table as everything else, so `set wp1_lat 52.1` works, `save` keeps them,
  the config protocol can write them, and `mission add` is a convenience that
  fills the next slot rather than a second kind of storage.
- **The pilot can always take it back.** Moving a stick past fifteen percent of
  travel stops the mission and hands the aircraft to the sticks. That is not a
  nicety: an autopilot nobody can interrupt is worse than no autopilot, and a
  stick is the fastest control in the aircraft.
- **A switch can start it, and end it.** `mission_channel` names a receiver
  channel - 7 by default in the simulated airframe, 0 meaning none - and the
  switch wins over the console while it is set: *going* high asks for the
  mission, going low ends it. It is the going high that asks, not the being
  high: a mission that something else stopped does not come back because the
  switch is still up, and a pilot who wants another one cycles the switch or
  types `mission start`. A mode that can only be selected from a laptop is a
  mode nobody selects in the air.
- **A failsafe outranks it.** The fence and a critical pack are judged *before*
  the mission and stop it: a pilot asked for a list of places to go, not for
  the aircraft to keep visiting them with a flat pack. The aircraft goes home
  instead, and does not take the mission back up when it is inside the fence
  again. This was wrong until it was measured - the mission block ran first and
  returned, so a pack that went critical under a mission left the aircraft
  flying waypoints: 275 m from home and still armed at the end of the session,
  with the pack at 3.25 V a cell (see the `flatpack` session in
  [18-software-in-the-loop.md](18-software-in-the-loop.md)).
- **The arm switch wins over everything.** Disarming ends a mission, as it ends
  anything else.
- **Losing the link does not end it.** The navigator was already flying; a
  mission that is going well should not turn into a failsafe because the radio
  dropped out. It keeps flying the list, and the console says `on autopilot`
  rather than `returning home` so the two are never confused.

The mission flies the same guidance as the return - the same course error, the
same altitude hold, the same loiter - with a target that moves when it is
reached. Past the end of the list it circles at the last waypoint, so a mission
whose last waypoint is home is a mission that ends at home.

### The same list on the quadrotor

The mission is the same code with the other profile, and it is worth saying
how much of it is *not* the same, because flying it in the loop is what found
two faults that the wing's mission could not show (`aerialkit-fw-sim 130
mission quad`, the eighth session, recorded in
[evidence/sil-quad-mission.txt](evidence/sil-quad-mission.txt)):

- **How close counts as arriving is the airframe's answer**, and it is the same
  answer for a waypoint as for home. A wing's arrival radius is the circle it
  can hold - tens of metres - and a quadrotor can stop *at* a point. The mission
  used `rth_arrive_m` for both, so the quadrotor "arrived" 45 m out and cut the
  corner of every leg. It uses `quad_arrive_m` now, and arrives 12 m from the
  first waypoint and 0 m from the second, on the same list the wing flies.
- **Arriving is not coming down.** The quadrotor's return descends when it
  reaches home, which is where the ground is: the altitude it descends to is
  the altitude *home* was captured at. A waypoint is somewhere else, so
  descending there means coming down onto whatever is below it at home's ground
  level - and with the descent gate open at every waypoint, the first version
  of this mission ended with a landing nobody asked for. The gate is the return
  now (`arrived && !flying_to_waypoint`); past the last waypoint the quadrotor
  holds station over it at the mission's altitude, which is what "the quad
  settles over it" always said it did.
- **And counting an arrival had a bug the quadrotor could reach and the wing
  could not.** The counter incremented on every step that was inside the radius
  and had not been the step before, which is right for an aircraft passing
  through and wrong for one *holding station on the radius*: drifting a metre
  out and back counts the same waypoint again. Measured: two waypoints, three
  arrivals. It counts each waypoint once now, by remembering which one it
  counted - and the *check* was as wrong as the counter: it asked whether the
  console ever said "2 reached", which a counter that goes 2 and then 3 passes
  on the way past. The wing's own committed transcript said "3 reached" for two
  waypoints while that check reported ok. It asserts two and not three now, for
  both airframes.

What the flown mission shows: both waypoints reached and counted, the flight
core reporting `on autopilot`, the altitude it started at held to the end
(3 m to 4 m), the aircraft holding station over the last waypoint instead of
flying on, and the transmitter switch handing it back to the sticks. A
quadrotor mission takes longer than a wing's - the same five hundred metres at
six metres a second instead of ten - which is why the session runs 130 seconds
and the list is not shortened to fit.

### And the wing's height had the same standing error the quadrotor's descent did

A wing sinks, so holding an altitude takes a standing nose-up - and a
proportional law can only get that from the altitude *error*, which means it
holds an altitude *below* the one it was given. Measured in the simulator
before any of this: the wing's return held 14 m where it was told to hold 15,
and a fence that asked it to climb to a 30 m floor settled at 26. Both are the
same droop, and both are gone now that the pitch the wing needs is *learned*:
the same return holds 14 and ends at 14, and the floor flight reaches 28 m of
its 30.

The host test is an A/B against a plant with a sink and a climb per unit of
pitch, and it is worth having because the arithmetic of the droop is what says
whether an integral is the right fix at all - the loop settles where 0.01 of
pitch per metre of error supplies the 0.05 it takes to stop the sink:

```text
altitude deficit: with the integral 0.12 m, without 5.00 m
```

The bound is a fifth of full pitch, and the integration stops while the output
is pinned and the error is still pushing it - the two rules the quadrotor's
vertical loop needed, for the same reasons. It is deliberately *slower* than
the quadrotor's: a wing's height is a phugoid, and an integral fast enough to
trim a quadrotor would pump that oscillation instead of damping it. The
parameter is `rth_alt_ki`, and `tests/test_nav.c:test_wing_altitude_droop` is
the A/B.

### And the mission a quadrotor could not start at all

Everything above is flown in the wind, and the wind was hiding a hole in the
whole idea. A quadrotor with no magnetometer learns which way its nose points
from the track it makes *through the air* - and an aircraft hovering in still
air makes no track at all. So a mission started from a hover in still air could
not begin: the navigator would not translate without a heading, and it could
not earn a heading without translating. Measured the first time it was asked
(`aerialkit-fw-sim 150 mission quad calm`): the aircraft hovered at **221 m
from its first waypoint** for the whole two minutes, with the yaw estimate
frozen 105 degrees from the nose. In wind it never showed, because the drift
gave the estimator a track - the *wind's* track, which is the bug the air-track
correction above fixes, and which had been quietly making the calm case
unreachable.

Two things were needed, and a third to make them work:

- **The nudge.** When the navigator has somewhere to go, nothing to aim with,
  and no motion, it makes a track to learn from: a third of a stick of forward
  tilt, held until the module's course means something, and capped at eight
  seconds - after which it holds and says so, because a shove that never earns
  a heading is an aircraft flying away from its mission for ever.
- **Straight, while it earns it.** The shove steers *nothing*: the course law
  is aimed by a heading nobody has, so a yaw command there is a turn toward
  somewhere the aircraft cannot know - and it is worse than useless while the
  heading is being earned, because a turning aircraft's track is not its nose,
  so the alignment chases a target that moves with it. Measured: with the
  steering left on, the shove reached three metres a second and the estimate
  ended up seventy degrees out, wandering.
- **And gently, once it has it.** The heading comes from the track, so a hard
  turn is a turn whose heading measurement lags the nose by the turn rate times
  the time the airframe takes to change velocity - and the tilt is computed in
  that lagging frame. The navigator's yaw is capped at a quarter of the stick
  for that reason. Measured without the cap: the estimate ended 160 degrees
  from the nose and the aircraft flew a circle two hundred metres across,
  *away* from where it was going.

With all three: the same calm mission reaches its first waypoint 4 m out and
its second 0 m out, counts both, holds station over the last one 6 m out, and
holds its altitude (3 m to 3 m). It is in `make test` as
`aerialkit-fw-sim 150 mission quad calm`, the extra time being what the nudge
costs at the start, and its transcript is
[evidence/sil-quad-mission-calm.txt](evidence/sil-quad-mission-calm.txt). The
unit halves are `tests/test_nav.c:test_alignment_nudge` (the shove, its
straightness, the climb-first rule and the cap) and the two checks in
`test_quad_return` that say an unaligned quadrotor never commands a roll.

The waypoint list is four long. That is a limit worth stating rather than
hiding: a configurator can fill it, `mission` prints it, and a longer mission
wants a longer table rather than a cleverer one.

## The floor and the fence

A return that arrives somewhere it should not have is what these numbers are
for, and between them they make a box: a floor under the altitude it holds, a
ring around home it will not leave, and a lid over it that is both.

**`rth_min_alt_m` is a floor under the altitude the navigator holds.** Zero is
none; anything else is the lowest altitude it will fly while it is flying. It
applies to a mission too, because a mission is the same guidance. What it is for
is the hill you were flying beside: a return that descends to whatever altitude
the aircraft happened to be at when the link went is a return that arrives
somewhere else, and the pilot is not there to notice.

The altitude it holds is the *fused* one, not the GPS's: a barometer's fast
changes with the GPS's absolute reference leaked in to stop its drift, which is
the difference between a wing holding a metre and a wing wandering through five
while the GPS catches up - [09-sensors.md](09-sensors.md). On a board with no
barometer the same code degenerates to the GPS altitude this firmware used
before there was one.

**`fence_enable` and `fence_radius_m` are a geofence**: fly further than that
from home and the aircraft comes back. It is the one automatic behaviour here
that takes an aircraft off a pilot whose link is still up, so it is off until
somebody turns it on, it announces itself on the console (`fence: 253 m from
home, bringing it back`), and it hands the aircraft back the moment it is inside
again. Enabling it is opting into the return it performs: a fence with
`rth_enable` off still flies the aircraft home, because a fence that does not
act is a fence that does nothing.

**And `fence_ceiling_m` is the same fence seen from below**: the highest the
aircraft may fly, in metres above home, zero for no lid. It is under the same
`fence_enable`, and it is two behaviours in one number.

- **It is a trigger**, for the reason the radius is one: a pilot flying by hand
  through the ceiling is a pilot who has left the box the same way, and somebody
  else has to take the aircraft. The console says `fence: 43 m up, bringing it
  back` - the height, where the radius message says the distance - and the
  navigator flies the same return it would have flown.
- **And it caps what the navigator will hold**, so a return that was engaged
  above the lid comes *down* rather than flying home high, which is the case a
  ceiling exists for: a legal altitude limit or a cloud base is not something
  the return on the way home is allowed to ignore.

Two things about it are worth knowing. The first is measured: the navigator
holds **ten metres under the ceiling, not at it**. Holding the number a trigger
fires at is a knife edge - the navigator takes the aircraft over the line, holds
it there, and every metre the vertical loop spends above its own target is
another breach. The fence session ran that way for one build and the pilot and
the navigator swapped control **four times in twenty seconds**, which is the
hazard the box above describes for a ring smaller than a wing's turn, one
dimension up. INAV's geozones carry the same idea as `geozone_safe_altitude_distance`
("vertical distance that must be maintained to the upper and lower limits of the
zone", ten metres by default), and that is where the number comes from.

The second is that the margin is taken out of the room the pilot has left
between the floor and the ceiling, so the two do not fight: a floor five metres
under the lid leaves five metres of margin and the floor is still what the
aircraft holds. A floor set *above* the ceiling is a pilot asking for something
impossible, and there the ceiling wins - a floor is terrain an aircraft can fly
over and a ceiling is airspace it may not be in. A lid so low that the margin
would put the hold underground holds the ground instead.

What a ceiling is **not** is a clamp on the pilot: the fence hands the aircraft
back as soon as it is under the lid again, and a pilot who climbs back through
it will be handed back to the navigator again. It is a fence, not an autopilot.

One thing about a fence is worth knowing before trusting one:

> **A fence smaller than the aircraft's turn radius will hand over and take back
> over, repeatedly.** A wing circling at a given bank has a radius - about 100 m
> at 15 m/s and 12 degrees - and it leaves the fence again on the far side of
> its own turn. The aircraft is safe in that loop, and the pilot never quite has
> it; the simulator's first fence scenario used a 150 m fence and a 12-degree
> turn and did exactly this. Set the fence wider than the circle the pilot is
> going to fly inside it, or the pilot will spend the flight being rescued.

## What the loop found

Two bugs, and both were invisible to every test that existed before the whole
firmware ran together. The wing scenario in
[18-software-in-the-loop.md](18-software-in-the-loop.md) is what found them; it
flies out under throttle, pulls the receiver, and checks that the aircraft comes
back.

**The navigator could never engage.** It decides whether to take over by asking
whether the pilot's link is up - and it asked *before* the flight core had
sensed the link for that step, so it was always reading the previous step's
answer. On the step the link actually went, the navigator still saw it up and
kept quiet, and the flight core latched the failsafe; on the next step the
navigator saw the link gone but the state was already FAILSAFE, which is not a
state it is allowed to take over from. Return-to-home was unreachable, and the
unit tests could not see it because they call the navigator directly, with no
flight core to be one step behind.

The fix is to sense the link once, before both of them read it:
`ak_flight_link_update()` is now called at the top of the loop and inside the
flight step, so the navigator and the core act on the same answer.

**The altitude gain was a thousand times too high.** `alt_kp` is "pitch per
metre of altitude error" - the header says so, the parameter table says so - but
the navigator multiplied it by the altitude error in *millimetres*. A tenth of a
metre of error was a full pitch command, so the altitude loop was not a loop at
all but a bang-bang controller, and the aircraft pitched between full nose-up
and full nose-down about once a second for the whole return. The unit test asked
which way the pitch went and never how far, so it passed. It now checks the
magnitude: twenty metres is a fifth of a command, one metre is a hundredth.

With both fixed, the simulated wing holds its captured altitude to within a
metre for the whole flight home.

## What is not

- **It has never flown.** The only evidence is the simulated one above: a
  point-mass airframe, no wind, no terrain, and a plant that is roughly a wing.
  The gains (`rth_course_kp`, `rth_alt_kp`, `rth_loiter_roll`, `rth_cruise`)
  are starting points, and the fact that they hold a simulation is the weakest
  kind of evidence there is.
- ~~The yaw axis fights the turn.~~ **Fixed** (2026-09-16). The yaw rate
  setpoint was zero, so the loop asked the motors to hold the nose still while
  the bank took the aircraft round a corner - one motor pegged, the other idled,
  for the whole return. It was easy to miss because the simulator's wing did not
  model differential thrust at all: making the plant model what the motors
  actually do turned it into a hard failure (with the coordination removed, the
  fence session's aircraft never comes back - 251 m at its closest, drifting to
  540 m), and the peak differential was **24.4 degrees a second** of yaw fought.
  The core now adds the turn the airframe is already making - `g tan(bank)`
  over the ground speed, which is the wing's business and the GPS's measurement,
  so it arrives as an input the way the guidance pointer does - and the same
  peak is **0.1 degrees a second**: the motors damp the turn instead of
  arguing with it. A quadrotor gets zero, because its yaw has nothing to do
  with its bank.
- ~~It is the wing profile only.~~ **A quadrotor's return is its own profile
  now** (2026-09-16) - climb, translate, settle - and the wing's checks above
  are the wing's. The next section is that profile.
- **Nothing checks that the way home is flyable.** No terrain, no altitude floor,
  no geofence: a wing returning home will fly through whatever is in the way. A
  minimum altitude and a fence are the next safety features, not this one.
- **Waypoints are not here.** Beyond return-to-home, the milestone wants a
  course to follow; that is the same guidance with a moving target.

## The quadrotor's return

The wing cannot stop, so its return ends in a circle. A quadrotor can, so its
return is a different manoeuvre, and it lives in its own profile
(`ak_nav_set_profile`): **climb, translate, settle**. Which one is flying is not
a parameter of its own - it follows `airframe`, because asking the pilot to keep
two selectors in step is asking for the one time they are not, and the console
prints which profile is armed.

Three things are properties of the airframe rather than preferences:

- **A tilt is an acceleration.** The profile commands a *velocity* - the
  position error, capped at `quad_return_speed`, scaled by `quad_return_kp` -
  and the tilt follows from the difference between that and the velocity the
  GPS reports. The damping is the half that makes it stable: the first version
  drove the tilt from the position error alone, and a test with no drag to slow
  it down found an aircraft that flies at home, past it, and back, forever.
- **The throttle is the vertical control**, around the hover throttle the pilot
  configured (`rth_cruise`). The climb rate is commanded from the altitude error
  and the throttle from the *rate* error, because the height has to be
  differentiated to get a rate - the GPS reports a ground speed and nothing
  vertical - and an error straight into throttle is a spring with nothing to
  damp it. That was the second thing the test found: the first version
  oscillated forty metres either side of the altitude it was asked to hold.
- **The yaw has to mean something.** A world-frame position error becomes a
  body-frame tilt through the heading, and this aircraft has no magnetometer.
  The heading comes from the GPS ground track, which is only good while the
  aircraft is moving - so the estimator aligns yaw to it slowly
  (`ak_estimator_aid_heading`) and reports whether that has happened, and the
  profile *climbs and holds* until it has. Translating on a yaw that is ninety
  degrees out is an aircraft flying confidently in the wrong direction.

The descent ends in a **landing**. The aircraft settles at `quad_hover_m` above
the ground home was set on, waits two seconds there - the hover is where the
estimate is allowed to settle and where the pilot can take it back - and then
comes down to the ground at no more than six tenths of a metre a second and
stops its motors.

The rule that decides "this is the ground" is deliberately not an altitude. The
estimate is a filtered barometer anchored by the GPS, and at a hundred metres
above sea level the simulator has it wobbling by metres: a rule that wanted
"within a metre of the ground" never fired at all, and one that wanted "within
five" would fire on a gust. What the aircraft can know instead is that it was
*told to come down* and *did not*: the navigator is in its landing phase, the
height has come down less than half a metre in two seconds while six tenths a
second was being asked for, and that has held for two windows in a row. The
flight core then has one rule, and it is the narrowest one it could be: an
aircraft a navigator is flying, which has landed, has finished flying. A pilot
holding the sticks never sees the rule at all.

That is also the only place this firmware stops its own motors, which is why it
is shaped the way it is: no rangefinder, no accelerometer transient, no motor
current - both boards have a barometer and that is the measurement - and four
seconds of evidence before anything is cut.

And because it reads a *stall*, it requires the height to be **measured**: the
barometer answering or the fix valid, the two sources the estimate is built
from. A sensor that has stopped leaves a perfectly still height, and the rule
must not mistake that for the ground - measured before this was added, with the
barometer failing as a descent began, the aircraft sat on the ground with its
motors still turning because the navigator's frozen height never reached the
hover band that opens the landing. [09-sensors.md](09-sensors.md) has the two
fixes and the numbers.

What is verified comes in two layers now. A host test flies a point mass
through the same code at 50 Hz: 300 m out, 50 m below the altitude it was asked
to hold, pointing east - it climbs first, comes home, descends to two metres
above home, and holds station there; and with `yaw_aligned` clear it climbs and
holds and does **not** translate. And since 2026-09-16 the simulator's own quad
has motion - a tilt is an acceleration, the collective is height, and the prop
pairs fight over the yaw - so the return is flown in the loop as
[evidence/sil-quad-return.txt](evidence/sil-quad-return.txt): out east on a
stick, the link goes at 66 m and 9 m/s, and the navigator brings it home to
within 3 m, descends to its hover height and holds station, with the mode the
handset was sent saying RTH and the pilot getting it back when the receiver
does.

That flown scenario found two things the host test could not, and both were
about the frame the tilt is computed in. **The yaw estimate had to be given a
use**: `ak_estimator_aid_heading` was written, tested on its own and *never
called* - nothing in the flight loop ran it - so the profile that refuses to
translate without an aligned heading would have climbed and hovered forever.
And **steering had to be part of translating**: a version that only translated,
with the nose wherever it happened to point, made a ground track at right
angles to the nose; the estimator aligned its yaw to that track, and the next
tilt came out of a frame that had just been turned underneath it. The aircraft
spun on the spot while travelling away from home at nine metres a second. The
profile now steers with the same course law the wing uses, aimed by the yaw
rate, so the nose turns toward home, the tilt follows the nose and the track
follows the velocity - which is also what makes the ground track a heading the
estimator can trust.

What is still not verified: a real quadrotor and the ground itself. Wind is
covered in the loop now (five metres a second, the next section), but it is a
constant horizontal uniform wind - the simplest one there is - and none of it
has happened to an airframe. The landing rule is checked in the loop against
the simulator's own altitude estimate - the same filtered barometer the
firmware has, not the simulator's private knowledge of where the ground is -
and the motors stop at about half a metre above home. On a real aircraft that
half metre is where a rangefinder would earn its place.

### What wind did to it, and what it turned out to be

The simulator had never had any wind in it, and adding five metres a second
shows the difference between where an aircraft points and where it goes. The
wing's return shrugs: it steers on the ground track, which is what a GPS
reports, and it comes home with its nose up to 86 degrees off its own track.

The quadrotor's return *appeared* to break: with wind, its altitude hunted by
about seventeen metres and it stopped short of home. The first explanation -
that the lean holding station against the wind made the mixer's authority limit
steal thrust from the vertical loop - was wrong, and the chase for it made the
calm case worse, which was the clue. **The fault was in the plant**, and it is
the kind of fault worth naming: the simulated quad had *no drag*, so a tilt set
an acceleration and a hover throttle set a climb *rate* that nothing opposed.
Both loops were double integrators, which no aircraft is, and the firmware's
gains - which assume a tilt sets a speed and a hover throttle holds a height,
the way any quadrotor does - were being asked to stabilise something that
behaves like a spacecraft. With drag in the plant (about two seconds to settle,
which is what the airframe's own numbers look like) the same return lands in
the same wind: it comes home, descends, and stops its motors on the ground,
staying disarmed when the receiver comes back.

### Where it comes down, and the two faults behind it, measured 2026-09-16

The first write-up of the wind case said the aircraft landed "up to twenty-five
metres downwind", and that number was read off the wrong moment: it is where
the *simulator's* arrival trigger fired - mid-descent, twenty-five metres out
and two and a half up - not where the aircraft stopped. The session prints where
it is once a second for the whole return, so the tape says what actually
happens ([evidence/sil-quad-return.txt](evidence/sil-quad-return.txt)), and it is
a worse picture in one place and a better one in another. This is the tape
*before* the two fixes below, which is why it is worth keeping:

| Time | What the plant is doing |
| --- | --- |
| 22.0 s | link lost, 105 m out, 22.3 m up, mode ANGLE |
| 42-43 s | crosses the 60 m *arrival* radius and starts descending from 23 m |
| 48.0 s | **on the ground, 38.5 m from home** - the descent ran out of distance before it ran out of height |
| 52.0 s | climbed back to 2.5 m, 24.9 m out |
| 61.2 s | motors stopped: 0.4 m above home, **2.5 m from home**, mode DISARM |

So the last landing *point* is good - on the pad, inside three metres, motors
stopped, disarmed - and the path to it is not: the aircraft reaches the ground
thirty-eight metres downwind and then flies the last of the distance towards
home with a few tenths of a metre of altitude under it. On a real airframe that
is a dragging arrival across whatever is between the two points, not a landing.

Two things were under suspicion and both were cleared by measurement, which is
how the third - the real one - was found:

- **The height estimate.** The suspicion was that the fused altitude lags the
  airframe during the descent, which would make any landing rule that reads it
  fire late. Measured with the aircraft's own console report (`baro`) sampled
  every two seconds through the descent, against the plant's true height:
  2.52/2.52, 1.50/1.49, 0.29/0.30, 0.06/0.07, 0.26/0.28 m - the estimate is
  within **two centimetres** of the truth, at every sample, all the way down.
- **The gains.** The vertical and translational loops were doing what they were
  tuned to do. They were given a target they could not reach, which is a
  different fault with a different fix.

### The three faults, and the tape that shows them

**One: the descent started at a wing's arrival radius.** Sixty metres is a
comfortable circle for a wing to hold while it waits; a quadrotor's arrival is
a *vertical descent*, so its radius is how far from home it can still be
descending. From 23 m up, crossing the radius at 2.8 m/s of ground speed, the
aircraft had 7.7 seconds of descent in hand and 21 seconds of distance to
cover, so it reached the ground with 38 m still to fly. The return has its own
arrival radius now - `quad_arrive_m`, fifteen metres by default - and the
descent starts where a quadrotor can hold station instead of where a wing can
orbit. The `settle` gate in `ak_nav.c` is that number, and it is deliberately
not `rth_arrive_m`.

**Two: the climb rate the vertical loop flew on was not a rate.** This is the
one that was actually making it dive, and it took a trace of the loop's own
numbers to see. The throttle is the vertical control and it follows the
*difference* between the climb rate the profile asks for and the one the
aircraft has - so the rate is a control input, and a wrong one is not a small
error but the wrong command. The rate was the height this pass minus the height
last pass, and the height arrives from the barometer at about 32 Hz as an
integer number of millimetres on a 1 kHz loop: unchanged six passes out of
seven, then a 33 mm jump. The trace says what that did - `climb=0.00 desired=-3.00
thr=0.100` on every sample, while the plant came down at 9.9 m/s. The loop
believed it was not descending, so it kept commanding descent, harder.

The rate is now measured over a window (`quad_climb_sample()`): eight samples
25 ms apart, which is a sixth of a second, five or six barometer updates, a
0.19 m/s quantum and under a tenth of a second of lag - a fraction of the
loop's own time constant, which is what keeps the phase margin. `tests/test_nav.c`
flies the last part of a return against a barometer quantised exactly that way
and checks what the aircraft did: with the per-step difference the descent
reaches 10.3 m/s and the aircraft goes through its hover height to the ground;
with the windowed rate it is 2.3 m/s against a three-metre-a-second cap and the
lowest it gets before the landing begins is 3.00 m above home, asked for 2.5.
The rate is the only thing that changed in the *vertical* loop: a faster
altitude-rate gain and a wider slow-descent band were both tried while chasing
this and both were dropped, because with the rate fixed they changed nothing a
measurement could see.

**Three: holding a station in wind needs to know the wind.** With the rate
fixed, the return came down over the pad but sat two and a half metres
downwind of it. That is not a tuning artifact, it is what a proportional
position loop does: a tilt accelerates the aircraft through the *air*, the GPS
reports it over the *ground*, and the difference between the two is a standing
force that only a standing position error can oppose. The loop integrates its
own position error into a standing addition to the velocity it asks for (the
term is `hold_n_m_s`/`hold_e_m_s` in `ak_nav.h`), conditioned not to integrate
while the loop is already asking for all the speed it has - on the way home the
error is large and *staying* large is exactly what a constant wind does. The session
measures it: the average distance from home while the aircraft settles is
**2.5 m without the term and 1.0 m with it**, and the motors stop 0.5 m from
home instead of 2.5 m.

What the loop does now, from the same session's tape (the plant's numbers, once
a second, in [evidence/sil-quad-return.txt](evidence/sil-quad-return.txt)):

| Time | What the plant is doing |
| --- | --- |
| 22.0 s | link lost, 136 m out, 22.3 m up, mode ANGLE |
| 65.7 s | starts coming down: 7.7 m from home, from the 22 m it was holding |
| 65.7-75.6 s | descends, about two metres a second and slowing as it closes |
| 75.6 s | first down at the hover height: 2 m from home, 3.5 m up |
| 76-87 s | settles, averaging 1.0 m from home against the wind |
| 87.2 s | motors stopped: 0.6 m above home, 0.5 m from home, disarmed |

The same session can be flown in still air - `aerialkit-fw-sim 100 quadrth
calm`, which is what the second word is for - and it is worth doing, because
the wind term above has to do nothing when there is no wind: the same return
then loses the link 46 m out at 4 m/s, starts down 5.4 m from home, settles
0.9 m from it and stops its motors 0.8 m from home. (The outbound leg is flown
by the aircraft rather than by the wind since this was written: a third of a
stick of nose-down, which is about four metres a second of its own airspeed
and 46 m of its own ground track. Before that the calm run drifted 15 m in
fifteen seconds and the wind session's 114 m was mostly the wind's doing.)

What is still not claimed: no part of it has happened to an airframe, the wind
is a constant 5 m/s with no gusts and no shear, and the sensors are ideal
unless a session asks otherwise - the 33 mm barometer staircase is the only
error in the tape this section reads, which is what made the climb-rate fault
findable and would bury it under a real part's noise. (`aerialkit-fw-sim 100
quadrth noisy` is the same return with that noise on: it lands 1 m from home,
and [18-software-in-the-loop.md](18-software-in-the-loop.md) has what it
found.)

### Losing the fix on the way home

A return exists because the pilot's link is gone, so the worst thing this
firmware can be handed is a return whose GPS module stops answering: nobody is
flying it, and the one measurement it cannot navigate on has just gone. What it
did before this was written is worth writing down rather than glossing, because
it was the wrong answer and nothing had measured it:

(The same holds for a fix that is *there* and not worth using - four satellites
with a 3D fix the receiver calls ok is the case INAV's `gps_min_sats` floor
exists for, and [13-gps.md](13-gps.md) has what the navigator does with one:
nothing, which is to say it holds, and drifts, until the fix is usable again.)

| Time | The plant, in the gps-loss session, before the fix |
| --- | --- |
| 20.0 s | link lost, 119 m out at 19.5 m up, mode RTH |
| 31.0 s | the console says the return is *engaged* |
| 32.0 s | the module goes quiet, 107 m from home at 20.2 m up |
| 34.0 s | **motors stopped** - the navigator gave up, the flight core saw a lost link with nobody flying and did the correct thing for a lost link |
| 34.1 s | 20.2 m, then falling; the aircraft is on the ground seconds later, 91 m from home |

Giving up is not a safe state to hand an aircraft to. The alternative is not
navigation - there is nothing to navigate with - it is **holding**, on the two
measurements that still work: the attitude estimate (gyro and accelerometer,
unaffected) and the height (barometer, unaffected). So the navigator flies
level and holds the altitude it was told to hold: for the wing that is straight
on at cruise throttle, for the quadrotor a hover that drifts with the wind.
Neither is a plan - a wing in that state flies until the battery is gone - but
both are better than a deliberate motor stop in the air, and both are visible:
the mode stays RTH or `on autopilot`, and the console counts the steps it has
been holding for (`held:  N steps with no fix, still holding - level, altitude
held`).

**What was missing here is now written and flown: a quadrotor with a rangefinder
comes down where it is.** A wing cannot - it has nowhere in particular to land
and no way to arrive there without a position - but a quadrotor can descend
onto the ground directly below it, and the part is what makes the last of that
descent a measurement rather than the same failed estimate. So a fourth
session, `aerialkit-fw-sim 130 gpslostland`, flies the identical flight (out on
a stick, the link gone, the module quiet at 32 s) with `quad_hold_land_s 8`
typed at the start, and the aircraft:

| Time | The plant, in the gps-loss landing |
| --- | --- |
| 20.0 s | link lost, 119 m out at 19.5 m up, mode RTH |
| 32.0 s | the module goes quiet, 20.2 m up |
| 40.0 s | eight seconds of holding is up, and the descent begins - the console says `held: ... and coming down where it is - the ground is being measured` |
| 104.9 s | the motors stop: **0.4 m up, 438 m from home** - which is downwind, because with no position there is nothing to hold it anywhere |

The lowest collective while the motors were turning was **0.51** against a 0.55
hover, so it is a descent the firmware commanded rather than a fall: 72.9
seconds of it, from 20.2 m down to the ground. The number that matters is the
distance: 438 m from home is the wind's doing over a minute of drifting with no
position fix, and the aircraft is *on the ground* wherever that is, disarmed,
with the pack it still has. The alternative was a hover until the pack was
flat, from twenty metres, at 438 m from home.

**And that 0.3 m/s was a bug, not a personality.** The first version of this
section said the descent settled at about 0.3 where the profile asked for 0.6,
and explained it as the honest shape of a proportional rate loop. It is the
honest shape of a proportional rate loop *with no integral*, and the fix is the
one the position loop already had: integrate the rate error that is left, and
let the integral hold the standing throttle the plant needs. Measured on a
plant with the simulator's own drag, the same descent flows twice:

```text
descent rate: with the integral 0.60 m/s, without 0.27, commanded 0.60
```

What it cost to get right is worth writing down, because both mistakes were
about the *bound* on what the integral may learn:

- A quarter of a hover throttle allowed is a bound the loop can sit on: the
  integral wound up during the fast part of a return - three metres a second
  down from altitude - and then took seconds to unwind when the profile
  changed its mind. The tape shows the aircraft carried straight through the
  hover height it was supposed to settle at: 3.1, 2.7, 2.4, 2.1, 1.6, 1.0,
  0.3, with no hover in it at all.
- So what it may hold is bounded **by what it is being asked for**: 0.12 of
  throttle per metre a second of commanded rate. A hover is then a state with
  nothing to learn - the throttle that holds a hover is the hover throttle, and
  the integral goes back to zero the moment the demand for a *rate* goes away -
  and a fast descent may learn what a fast descent needs and must give it back
  when the demand falls. The proportional term pays for the rest with a
  temporary error it forgets.

The landing is unaffected in kind and better in detail: the descent is now the
half metre a second the profile asks for rather than whatever a standing error
produced, the motor stop moved from 0.38 m to 0.32 m in the gps-loss landing,
and the whole descent from 19.8 m took 44.3 s instead of 72.9. The unit half is
`tests/test_nav.c:test_vertical_rate_loop`, which flies the same descent twice -
with the integral and without it - and asserts both numbers.

And the same session with the part taken off (`gpslostland norange`) is the
negative control: the pilot's setting alone must not bring an aircraft down on
an instrument that has failed, so it holds, and the motors keep turning. Both
runs are in `make test`, and the unit half - the descent keeps going while the
part says there is ground left below, and the hold becomes a descent only with
a part and only after the delay - is `tests/test_nav.c:test_rangefinder_descent`.

After the fix, the same session: the module goes quiet at 32.0 s with the
return engaged and the aircraft at 20.2 m, and for the next 38 seconds the
motors keep turning (lowest collective **0.53** against a hover of 0.55), the
altitude stays at **20.2 m** - the level it was holding - and the handset goes
on being told RTH. What it does not do is stay over home: with no position
reference the loop has nothing to hold a position against, so the aircraft
drifts with the wind instead. That is the honest shape of this behaviour, and
the reason the landing it cannot do is a separate job.

The unit test is `tests/test_nav.c:test_gps_loss_holds`, which is narrow on
purpose: with `gps_valid` false the command must come from the altitude and the
attitude and *not* from the position, because the position in that input is
stale - and the same input with the flag set does steer, which is what proves
the flag is what decides. Everything else follows from `main()` passing the fix's
validity in and keeping the navigator engaged while it holds.
