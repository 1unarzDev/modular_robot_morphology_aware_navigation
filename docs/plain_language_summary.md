# Where can a modular robot change shape? (plain-language summary)

Draft for non-specialist readers (applications, fair judges). Every number
comes from the committed study records; the real-buildings sentence is filled
only once that study's report exists.

**The problem.** A modular robot can split into wheeled pods and reassemble
in a new shape, for example a wide, stable body for open rooms and a long,
narrow one for squeezing through a doorway. Earlier work treats the change of
shape as a switch: if both shapes fit at a spot, the robot can change there.
But changing shape is a physical event. Pods detach, drive around each other
and dock again; the robot then checks its new body by wiggling; and docking
needs a fixed camera in the room to see the pods precisely. A spot where both shapes fit can
still fail any of those steps.

**What I built.** A planner that chooses the route, the shape, and *where* to
change shape together, and only accepts a spot if the pods' 3D paths are
clear, the check-wiggle is clear, and the docking camera can see the pods. I
compared it against planners that check less, on the same simulated robot,
sensors and rooms.

**How I tested it.** I fixed the design, sample size and analysis rules in
advance, then ran 720 full simulated missions in 60 generated room layouts,
pairing every method on identical rooms.

**What I found.** My planner completed 167 of 180 missions. A planner that
only checked that both shapes fit completed 45 of 180, and one that also
checked the pods' paths and the wiggle, but not whether the camera could see
the dock, completed 90. Measured as the study
declared in advance (averaged over room layouts, and against the
feasibility-only planner only in the rooms where docking visibility can
matter), the improvements were 68 and 64 percentage points, far beyond the
smallest effect the study was designed to detect (p = 0.00002). Every failure of the
simpler planners was caused by exactly the check they skipped. And none of
my planner's 336 shape changes failed.

**Does it matter in real buildings?** In 300 randomly furnished generated
rooms, the simpler planners' chosen spot failed my planner's full check about 5%
of the time, always because the camera could not see the dock.
REAL-BUILDINGS RESULT: in 900 scanned real homes, offices and shops, ...

**What I learned.** Changing shape is not free, and where you do it matters
as much as whether you can. I also learned how easily a measurement bug can
look like a real result: an early timing bug made the robot seem to fail one
time in four. I traced it to the test harness, fixed it, and corrected
the project's evidence record before running the final study.
