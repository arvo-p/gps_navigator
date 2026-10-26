/*---------------------------------------------------------------------------/
/  FatFs - FAT file system module configuration file (R0.12c)
/  Configured with exFAT (_FS_EXFAT = 1) and LFN (_USE_LFN = 1) enabled
/---------------------------------------------------------------------------*/

#ifndef _FFCONF
#define _FFCONF 68300	/* Revision ID */

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define _FS_READONLY	0
#define _FS_MINIMIZE	0
#define _USE_STRFUNC	1
#define _USE_FIND		1
#define _USE_MKFS		0
#define _USE_FASTSEEK	1
#define _USE_EXPAND		0
#define _USE_CHMOD		0
#define _USE_LABEL		0
#define _USE_FORWARD	0

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define _CODE_PAGE	850
#define _USE_LFN	1
#define _MAX_LFN	255
#define _LFN_UNICODE	0
#define _STRF_ENCODE	3
#define _FS_RPATH	0

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/

#define _VOLUMES	1
#define _STR_VOLUME_ID	0
#define _VOLUME_STRS	"SD"
#define _MULTI_PARTITION	0
#define _MIN_SS		512
#define _MAX_SS		512
#define _USE_TRIM	0
#define _FS_NOFSINFO	0

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define _FS_TINY	0
#define _FS_EXFAT	1
#define _FS_NORTC	1
#define _NORTC_MON	10
#define _NORTC_MDAY	20
#define _NORTC_YEAR	2026
#define _FS_LOCK	0
#define _FS_REENTRANT	0

#endif /* _FFCONF */
