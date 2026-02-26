// Heartbeat service - periodic LED toggle to indicate the system is alive
#ifndef SRV_HEARTBEAT_H
#define SRV_HEARTBEAT_H

void srvHeartbeatSetup(int ledId, int pin);
void srvHeartbeatTask(void);

#endif
