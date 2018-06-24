
#ifndef __NODE_ID_H__
#define __NODE_ID_H__

void node_id_restore(void);
void node_id_burn(unsigned short node_id);

extern unsigned short node_id;
extern unsigned char node_mac[8];

#endif /* __NODE_ID_H__ */
