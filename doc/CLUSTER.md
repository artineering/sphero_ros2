# ROS2 Cluster Infrastructure

## Cluster Topology

```
[Internet] <--WiFi--> [RPi5 rpi5-main] <--Ethernet--> [PoE+ Switch] <--Ethernet--> [RPi4 node01..04]
                        10.0.0.1                                        10.0.0.11-14
```

| Node | Hostname | IP | Role |
|------|----------|-----|------|
| RPi5 | rpi5-main | 10.0.0.1 | Gateway, DDS server, NFS server, main dev node |
| RPi4 #1 | rpi4-node01 | 10.0.0.11 | Leaf node (4 Sphero instances) |
| RPi4 #2 | rpi4-node02 | 10.0.0.12 | Leaf node (4 Sphero instances) |
| RPi4 #3 | rpi4-node03 | 10.0.0.13 | Leaf node (4 Sphero instances) |
| RPi4 #4 | rpi4-node04 | 10.0.0.14 | Leaf node (4 Sphero instances) |

## System Details

- **OS:** Ubuntu 24.04 LTS (Noble) on all nodes
- **ROS2:** Rolling (packages from noble repo)
- **DDS:** Cyclone DDS with peer-based discovery
- **RPi5:** Boots from 256GB NVMe SSD
- **RPi4s:** Boot from SD cards, powered via PoE+
- **Username:** svaghela (all nodes)
- **SSH:** Passwordless key-based auth from RPi5 to all nodes

## Network Configuration

- **RPi5 WiFi:** Connects to "ghibli" network (internet uplink)
- **RPi5 eth0:** Static 10.0.0.1/24, NAT gateway via nftables
- **RPi4 eth0:** Static 10.0.0.1X/24, gateway 10.0.0.1
- **DNS:** 8.8.8.8, 8.8.4.4

## NFS Shared Workspace

The sphero_ros2 workspace is hosted on RPi5's SSD and shared to all RPi4 nodes via NFS:

- **Export:** /home/svaghela/sphero_ros2 to 10.0.0.0/24
- **Mount:** All RPi4 nodes mount at /home/svaghela/sphero_ros2
- **Persistent:** Configured in /etc/fstab on each RPi4

Build once on RPi5, run anywhere.

## Cyclone DDS Configuration

Config files at /etc/cyclonedds/cyclonedds.xml on all nodes.

- **RPi5:** Lists all leaf peers (10.0.0.11-14), ParticipantIndex=0
- **RPi4s:** Peer points to 10.0.0.1 only, ParticipantIndex=auto
- **Interface:** eth0 on all nodes
- **ROS_DOMAIN_ID:** 0

## Environment Variables (all nodes, in ~/.bashrc)

```bash
source /opt/ros/rolling/setup.bash
source ~/sphero_ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
export ROS_DOMAIN_ID=0
```

## SSH Access

From RPi5 to any node:
```bash
ssh svaghela@10.0.0.11  # node01
ssh svaghela@10.0.0.12  # node02
ssh svaghela@10.0.0.13  # node03
ssh svaghela@10.0.0.14  # node04
```

Run commands on all nodes:
```bash
for ip in 11 12 13 14; do ssh svaghela@10.0.0.$ip "<command>"; done
```

## Monitoring

Foxglove Bridge runs on RPi5:
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
Connect Foxglove Studio to: ws://rpi5-main.local:8765

## Distributed Launch Strategy

The intended distribution of nodes across the cluster:

### RPi5 (rpi5-main) runs:
- multirobot_webserver (multirobot_webapp) - central dashboard
- foxglove_bridge - monitoring
- aruco_slam - camera-based localization (if camera attached)
- soccer_game_controller - game orchestration

### Each RPi4 runs (4 Sphero instances per node):
- 4x sphero_instance_device_controller_node (one per Sphero)
- 4x sphero_instance_task_controller_node (one per Sphero)
- 4x sphero_instance_statemachine_controller_node (one per Sphero)
- 4x sphero_instance_websocket_server (one per Sphero)

Each instance is namespaced by sphero_name parameter (e.g., SB-3660, SB-74FB, etc.)

## Key Infrastructure Files

- /etc/netplan/50-cloud-init.yaml - eth0 static IP
- /etc/netplan/90-cluster-wifi.yaml - WiFi (RPi5 only)
- /etc/cyclonedds/cyclonedds.xml - DDS discovery config
- /etc/nftables-cluster.conf - NAT rules (RPi5 only)
- /etc/fstab - NFS mount (RPi4s)
- /etc/exports - NFS export (RPi5)

## Troubleshooting

### Node not reachable
```bash
ping 10.0.0.XX
ip neigh show dev eth0
# If FAILED: check RPi4 wired connection settings (must be manual, not DHCP)
```

### NFS mount empty
```bash
sudo mount 10.0.0.1:/home/svaghela/sphero_ros2 /home/svaghela/sphero_ros2
```

### Read-only filesystem on RPi5
```bash
sudo mount -o remount,rw /dev/nvme0n1p2 /
# Root cause: /etc/fstab must use UUID, not LABEL
```

### ROS2 nodes not discovering each other
```bash
# Verify DDS config
echo $CYCLONEDDS_URI
cat /etc/cyclonedds/cyclonedds.xml
# Test with talker/listener
ros2 run demo_nodes_cpp talker  # on one node
ros2 run demo_nodes_cpp listener  # on another
```
