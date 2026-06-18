# hal is bundle-agnostic and carries NO raccoon-transport dependency: the
# transport publish for the heartbeat lives behind the extern "C"
# raccoon_platform_heartbeat_publish symbol provided by the selected platform
# bundle, and ScreenRender's transport member moved into the wombat bundle TU.
# Keeping hal transport-free is what lets one hal extension load against either
# the mock or the wombat platform .so at runtime.
register_libstp_module(hal "FALSE" "foundation;threading;kinematics")
