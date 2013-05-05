/*
 * Copyright (c) 2010-2012 Red Hat, Inc. All rights reserved.
 * Author: David Chinner
 *
 * Generic LRU infrastructure
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list_lru.h>

int
list_lru_add(
	struct list_lru	*lru,
	struct list_head *item)
{
	spin_lock(&lru->lock);
	if (list_empty(item)) {
		list_add_tail(item, &lru->list);
		lru->nr_items++;
		spin_unlock(&lru->lock);
		return 1;
	}
	spin_unlock(&lru->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(list_lru_add);

int
list_lru_del(
	struct list_lru	*lru,
	struct list_head *item)
{
	spin_lock(&lru->lock);
	if (!list_empty(item)) {
		list_del_init(item);
		lru->nr_items--;
		spin_unlock(&lru->lock);
		return 1;
	}
	spin_unlock(&lru->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(list_lru_del);

long
list_lru_walk(
	struct list_lru *lru,
	list_lru_walk_cb isolate,
	void		*cb_arg,
	long		nr_to_walk)
{
	struct list_head *item, *n;
	long removed = 0;
restart:
	spin_lock(&lru->lock);
	list_for_each_safe(item, n, &lru->list) {
		int ret;

		if (nr_to_walk-- < 0)
			break;

		ret = isolate(item, &lru->lock, cb_arg);
		switch (ret) {
		case 0:	/* item removed from list */
			lru->nr_items--;
			removed++;
			break;
		case 1: /* item referenced, give another pass */
			list_move_tail(item, &lru->list);
			break;
		case 2: /* item cannot be locked, skip */
			break;
		case 3: /* item not freeable, lock dropped */
			goto restart;
		default:
			BUG();
		}
	}
	spin_unlock(&lru->lock);
	return removed;
}
EXPORT_SYMBOL_GPL(list_lru_walk);

long
list_lru_dispose_all(
	struct list_lru *lru,
	list_lru_dispose_cb dispose)
{
	long disposed = 0;
	LIST_HEAD(dispose_list);

	spin_lock(&lru->lock);
	while (!list_empty(&lru->list)) {
		list_splice_init(&lru->list, &dispose_list);
		disposed += lru->nr_items;
		lru->nr_items = 0;
		spin_unlock(&lru->lock);

		dispose(&dispose_list);

		spin_lock(&lru->lock);
	}
	spin_unlock(&lru->lock);
	return disposed;
}

int
list_lru_init(
	struct list_lru	*lru)
{
	spin_lock_init(&lru->lock);
	INIT_LIST_HEAD(&lru->list);
	lru->nr_items = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(list_lru_init);
