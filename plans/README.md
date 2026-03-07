# Plans Directory

This directory stores execution plans created by the ROS2 Expert Agent before implementing changes to the workspace.

## Purpose

All ROS2-related tasks should follow a plan-first approach:
1. Create a detailed plan
2. Save it here with timestamp
3. Get user approval
4. Execute the plan
5. Update plan status

## Naming Convention

Plans should follow this naming pattern:
```
<task-description>-<ISO-8601-timestamp>.md
```

Examples:
- `add-battery-service-2026-03-07T14-30-00.md`
- `fix-multi-robot-namespace-2026-03-07T15-45-30.md`
- `implement-waypoint-navigation-2026-03-08T09-00-00.md`

## Plan Structure

Each plan should include:
- **Title** - Clear task description
- **Timestamp** - When plan was created
- **Task Description** - What needs to be done
- **Analysis** - Current state assessment
- **Detailed Plan** - Step-by-step execution plan
- **Expected Outcomes** - What success looks like
- **Potential Risks** - Things to watch out for
- **Testing Plan** - How to verify it works
- **Approval Status** - Checkboxes for workflow tracking

## Using the ROS2 Expert Agent

To invoke the ROS2 expert agent:
```
/agent ros2_expert
```

Then describe your ROS2 task. The agent will automatically:
1. Create a plan
2. Save it in this directory
3. Ask for your approval
4. Execute upon confirmation

## Benefits

- **Traceability** - Complete history of planned changes
- **Review** - Opportunity to catch issues before implementation
- **Documentation** - Plans serve as implementation documentation
- **Learning** - Review past plans to understand decision-making

## Maintenance

- Plans can be moved to an `archive/` subdirectory after execution
- Keep recent plans accessible for reference
- Delete plans that were rejected and won't be implemented
