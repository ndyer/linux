#ifndef _RMI_COMPAT_H_
#define _RMI_COMPAT_H_

#include <linux/input/mt.h>

#define INPUT_PROP_TOPBUTTONPAD         0x04
#define MT_TOOL_PALM			MT_TOOL_FINGER

/* this will leak since the new string is never freed */
#define devm_kasprintf(dev, gfp, fmt, ...) kasprintf(gfp, fmt, __VA_ARGS__)
#define input_mt_assign_slots(dev, slots, pos, num_pos, dmax) \
				input_mt_assign_slots(dev, slots, pos, num_pos)

#endif
