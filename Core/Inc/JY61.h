#ifndef __JY61_H
#define __JY61_H

struct SAcc
{
	volatile short a[3];
	volatile short T;
};
struct SGyro
{
	volatile short w[3];
	volatile short T;
};
struct SAngle
{
	volatile short Angle[3];
	volatile short T;
};
 
extern char YAWCMD[3];
extern char ACCCMD[3];
extern char SLEEPCMD[3];
extern char UARTMODECMD[3];
extern char IICMODECMD[3];
								
#endif
