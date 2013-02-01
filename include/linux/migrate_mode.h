#ifndef MIGRATE_MODE_H_INCLUDED
#define MIGRATE_MODE_H_INCLUDED
/*
 * MIGRATE_ASYNC means never block
 * MIGRATE_SYNC_LIGHT in the current implementation means to allow blocking
 *	on most operations but not ->writepage as the potential stall time
 *	is too significant
 * MIGRATE_SYNC will block when migrating pages
 * MIGRATE_FAULT called from the fault path to migrate-on-fault for mempolicy
 *	this path has an extra reference count
 */
enum migrate_mode {
	MIGRATE_ASYNC,
	MIGRATE_SYNC_LIGHT,
	MIGRATE_SYNC,
	MIGRATE_FAULT,
};

#endif		/* MIGRATE_MODE_H_INCLUDED */
