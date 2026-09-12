# Separate hybrid search from detailed docking physics

Search uses calibrated time, energy, and failure-risk models for certified transitions, while Gazebo executes detailed detach and redock physics only for the selected plan. This preserves realistic evaluation without placing contact simulation in the planner's inner loop.

