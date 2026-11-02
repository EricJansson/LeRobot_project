"""SO-101 imports shared by LeRobot 0.4 and 0.6 installations."""

try:
    from lerobot.robots.so101_follower.so101_follower import SO101Follower as UpstreamSO101Follower
    from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
except ModuleNotFoundError as exc:
    # Missing transitive dependencies must remain visible, rather than being
    # mistaken for a different LeRobot package layout.
    if exc.name not in {
        "lerobot.robots.so101_follower",
        "lerobot.robots.so101_follower.so101_follower",
        "lerobot.robots.so101_follower.config_so101_follower",
    }:
        raise
    from lerobot.robots.so_follower import SO101Follower as UpstreamSO101Follower
    from lerobot.robots.so_follower import SO101FollowerConfig
