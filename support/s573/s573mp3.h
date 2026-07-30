// System 573 Digital I/O MP3 service (HPS side).
// Poll hook, called from user_io_poll() alongside mdplus_poll().
#ifndef S573MP3_H
#define S573MP3_H

void s573mp3_poll();
void s573mp3_reset();   // called on core load / ROM change

#endif
