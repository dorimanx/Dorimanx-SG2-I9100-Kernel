/*
   EDS loop encryption enabling module

   Copyright (C)  2012 Oleg <oleg@sovworks.com>

   This module is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This module is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this module; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef EDS_KM_H
#define	EDS_KM_H

#include <linux/ioctl.h>

#ifdef	__cplusplus
extern "C" {
#endif
	
#define EDS_SECTOR_SIZE 512
#define EDS_KEY_SIZE 64

#define EDS_PROC_ENTRY "eds"

#define LO_CRYPT_EDS 17

struct eds_key_info
{
	int key_size;
	unsigned char *key;
};

	
#define EDS_CMD_SET_KEY 1

#define EDS_CTL_CODE 'e'
#define EDS_SET_KEY _IOW(EDS_CTL_CODE,EDS_CMD_SET_KEY,struct eds_key_info)


#ifdef	__cplusplus
}
#endif

#endif	/* EDS_KM_H */

