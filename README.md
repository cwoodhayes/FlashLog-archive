This repo is my (conor's) quick archive of the FlashLog memory management layer I wrote in 2018, copied over from RPL's private repo so I can show it to people if I want :)

Quick info:
- Sits on top of a flash memory driver (we used a NOR flash in flight), and enables writing telemetry & system state to the memory in the form of packets
- Packets are written with checksums for error detection--since this was used on a space vehicle, in which bit errors are common due to radiation exposure
- Flashlog can quickly recover system state in log(n) time using binary search in case of power outage
- Flashlog can dump its contents for data recovery purposes. Packets are parsed by an accompanying python script.
