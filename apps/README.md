# Apps

End-to-end Copper example apps live here, one directory per app, each with its
own `copperconfig.ron` and `justfile` importing the root one.

Apps are how roadmap phases prove themselves: per [ROADMAP.md](../ROADMAP.md),
a phase is not done until a runnable app in this directory demonstrates it
end-to-end (robot description in, planned and executed trajectory out).

No app exists yet. The first one arrives with Phase 1 (trajectory generation
and execution): URDF import -> RRT* plan -> timed trajectory -> simulated
execution under collision monitoring.
