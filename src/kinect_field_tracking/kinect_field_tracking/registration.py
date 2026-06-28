#!/usr/bin/env python3
"""Pure registration helpers (no ROS): target selection + status merge.

Kept ROS-free so the registration bookkeeping is unit-testable without a graph.
"""


def fresh_deployed(last_seen, now, heartbeat_fresh_sec):
    """Names whose heartbeat is fresh: (now - last_seen) <= heartbeat_fresh_sec.

    `last_seen` is {name -> last_seen_epoch}. Matches the webserver's
    HEARTBEAT_FRESH_SEC (default 15s) notion of "deployed".
    """
    out = []
    for name, ls in last_seen.items():
        if ls is None:
            continue
        if (now - float(ls)) <= heartbeat_fresh_sec:
            out.append(name)
    return sorted(out)


def select_targets(requested, last_seen, now, heartbeat_fresh_sec):
    """Targets to register: explicit `requested` if non-empty, else fresh-deployed."""
    if requested:
        return list(requested)
    return fresh_deployed(last_seen, now, heartbeat_fresh_sec)


def merge_registration_status(prev_registered, prev_failed, targets,
                              new_registered, new_failed):
    """Merge a subset-scoped pass into the full registration status.

    Callsigns NOT in `targets` keep their previous green/red; callsigns in
    `targets` take their new outcome. Returns (registered_sorted, failed_sorted),
    each a de-duplicated sorted list with no name in both.
    """
    targets = set(targets)
    new_reg = set(new_registered)
    new_fail = set(new_failed)

    registered = {n for n in prev_registered if n not in targets} | new_reg
    failed = {n for n in prev_failed if n not in targets} | new_fail
    # a freshly-registered name must not linger in failed
    failed -= registered
    return sorted(registered), sorted(failed)


def registration_status_payload(registered, failed):
    """The latched ~/registration_status JSON dict."""
    return {
        'complete': True,
        'registered': sorted(registered),
        'failed': sorted(failed),
    }
