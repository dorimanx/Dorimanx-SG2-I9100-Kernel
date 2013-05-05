/*
 * Copyright (C) 2007-2011 B.A.T.M.A.N. contributors:
 *
 * Marek Lindner, Simon Wunderlich
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 *
 */

#include "main.h"
#include "translation-table.h"
#include "soft-interface.h"
#include "hard-interface.h"
#include "hash.h"
#include "originator.h"

static void tt_local_purge(struct work_struct *work);
static void _tt_global_del_orig(struct bat_priv *bat_priv,
				 struct tt_global_entry *tt_global_entry,
				 char *message);

/* returns 1 if they are the same mac addr */
static int compare_ltt(struct hlist_node *node, void *data2)
{
	void *data1 = container_of(node, struct tt_local_entry, hash_entry);

	return (memcmp(data1, data2, ETH_ALEN) == 0 ? 1 : 0);
}

/* returns 1 if they are the same mac addr */
static int compare_gtt(struct hlist_node *node, void *data2)
{
	void *data1 = container_of(node, struct tt_global_entry, hash_entry);

	return (memcmp(data1, data2, ETH_ALEN) == 0 ? 1 : 0);
}

static void tt_local_start_timer(struct bat_priv *bat_priv)
{
	INIT_DELAYED_WORK(&bat_priv->tt_work, tt_local_purge);
	queue_delayed_work(bat_event_workqueue, &bat_priv->tt_work, 10 * HZ);
}

static struct tt_local_entry *tt_local_hash_find(struct bat_priv *bat_priv,
						   void *data)
{
	struct hashtable_t *hash = bat_priv->tt_local_hash;
	struct hlist_head *head;
<<<<<<< HEAD
	struct hlist_node *node;
	struct tt_local_entry *tt_local_entry, *tt_local_entry_tmp = NULL;
	int index;
=======
	struct batadv_tt_common_entry *tt_common_entry;
	struct batadv_tt_common_entry *tt_common_entry_tmp = NULL;
	uint32_t index;
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators

	if (!hash)
		return NULL;

	index = choose_orig(data, hash->size);
	head = &hash->table[index];

	rcu_read_lock();
<<<<<<< HEAD
	hlist_for_each_entry_rcu(tt_local_entry, node, head, hash_entry) {
		if (!compare_eth(tt_local_entry, data))
=======
	hlist_for_each_entry_rcu(tt_common_entry, head, hash_entry) {
		if (!batadv_compare_eth(tt_common_entry, data))
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
			continue;

		tt_local_entry_tmp = tt_local_entry;
		break;
	}
	rcu_read_unlock();

	return tt_local_entry_tmp;
}

static struct tt_global_entry *tt_global_hash_find(struct bat_priv *bat_priv,
						     void *data)
{
	struct hashtable_t *hash = bat_priv->tt_global_hash;
	struct hlist_head *head;
	struct hlist_node *node;
	struct tt_global_entry *tt_global_entry;
	struct tt_global_entry *tt_global_entry_tmp = NULL;
	int index;

	if (!hash)
		return NULL;

	index = choose_orig(data, hash->size);
	head = &hash->table[index];

	rcu_read_lock();
	hlist_for_each_entry_rcu(tt_global_entry, node, head, hash_entry) {
		if (!compare_eth(tt_global_entry, data))
			continue;

		tt_global_entry_tmp = tt_global_entry;
		break;
	}
	rcu_read_unlock();

	return tt_global_entry_tmp;
}

int tt_local_init(struct bat_priv *bat_priv)
{
	if (bat_priv->tt_local_hash)
		return 1;

	bat_priv->tt_local_hash = hash_new(1024);

	if (!bat_priv->tt_local_hash)
		return 0;

	atomic_set(&bat_priv->tt_local_changed, 0);
	tt_local_start_timer(bat_priv);

	return 1;
}

void tt_local_add(struct net_device *soft_iface, uint8_t *addr)
{
	struct bat_priv *bat_priv = netdev_priv(soft_iface);
	struct tt_local_entry *tt_local_entry;
	struct tt_global_entry *tt_global_entry;
	int required_bytes;

	spin_lock_bh(&bat_priv->tt_lhash_lock);
	tt_local_entry = tt_local_hash_find(bat_priv, addr);
	spin_unlock_bh(&bat_priv->tt_lhash_lock);

	if (tt_local_entry) {
		tt_local_entry->last_seen = jiffies;
		return;
	}

	/* only announce as many hosts as possible in the batman-packet and
	   space in batman_packet->num_tt That also should give a limit to
	   MAC-flooding. */
	required_bytes = (bat_priv->num_local_tt + 1) * ETH_ALEN;
	required_bytes += BAT_PACKET_LEN;

	if ((required_bytes > ETH_DATA_LEN) ||
	    (atomic_read(&bat_priv->aggregated_ogms) &&
	     required_bytes > MAX_AGGREGATION_BYTES) ||
	    (bat_priv->num_local_tt + 1 > 255)) {
		bat_dbg(DBG_ROUTES, bat_priv,
			"Can't add new local tt entry (%pM): "
			"number of local tt entries exceeds packet size\n",
			addr);
		return;
	}

	bat_dbg(DBG_ROUTES, bat_priv,
		"Creating new local tt entry: %pM\n", addr);

	tt_local_entry = kmalloc(sizeof(struct tt_local_entry), GFP_ATOMIC);
	if (!tt_local_entry)
		return;

	memcpy(tt_local_entry->addr, addr, ETH_ALEN);
	tt_local_entry->last_seen = jiffies;

	/* the batman interface mac address should never be purged */
	if (compare_eth(addr, soft_iface->dev_addr))
		tt_local_entry->never_purge = 1;
	else
		tt_local_entry->never_purge = 0;

	spin_lock_bh(&bat_priv->tt_lhash_lock);

	hash_add(bat_priv->tt_local_hash, compare_ltt, choose_orig,
		 tt_local_entry, &tt_local_entry->hash_entry);
	bat_priv->num_local_tt++;
	atomic_set(&bat_priv->tt_local_changed, 1);

	spin_unlock_bh(&bat_priv->tt_lhash_lock);

	/* remove address from global hash if present */
	spin_lock_bh(&bat_priv->tt_ghash_lock);

	tt_global_entry = tt_global_hash_find(bat_priv, addr);

	if (tt_global_entry)
		_tt_global_del_orig(bat_priv, tt_global_entry,
				     "local tt received");

	spin_unlock_bh(&bat_priv->tt_ghash_lock);
}

int tt_local_fill_buffer(struct bat_priv *bat_priv,
			  unsigned char *buff, int buff_len)
{
<<<<<<< HEAD
	struct hashtable_t *hash = bat_priv->tt_local_hash;
	struct tt_local_entry *tt_local_entry;
	struct hlist_node *node;
	struct hlist_head *head;
	int i, count = 0;
=======
	struct batadv_priv *bat_priv = netdev_priv(soft_iface);
	struct batadv_tt_local_entry *tt_local;
	struct batadv_tt_global_entry *tt_global;
	struct hlist_head *head;
	struct batadv_tt_orig_list_entry *orig_entry;
	int hash_added;
	bool roamed_back = false;

	tt_local = batadv_tt_local_hash_find(bat_priv, addr);
	tt_global = batadv_tt_global_hash_find(bat_priv, addr);

	if (tt_local) {
		tt_local->last_seen = jiffies;
		if (tt_local->common.flags & BATADV_TT_CLIENT_PENDING) {
			batadv_dbg(BATADV_DBG_TT, bat_priv,
				   "Re-adding pending client %pM\n", addr);
			/* whatever the reason why the PENDING flag was set,
			 * this is a client which was enqueued to be removed in
			 * this orig_interval. Since it popped up again, the
			 * flag can be reset like it was never enqueued
			 */
			tt_local->common.flags &= ~BATADV_TT_CLIENT_PENDING;
			goto add_event;
		}

		if (tt_local->common.flags & BATADV_TT_CLIENT_ROAM) {
			batadv_dbg(BATADV_DBG_TT, bat_priv,
				   "Roaming client %pM came back to its original location\n",
				   addr);
			/* the ROAM flag is set because this client roamed away
			 * and the node got a roaming_advertisement message. Now
			 * that the client popped up again at its original
			 * location such flag can be unset
			 */
			tt_local->common.flags &= ~BATADV_TT_CLIENT_ROAM;
			roamed_back = true;
		}
		goto check_roaming;
	}

	tt_local = kmalloc(sizeof(*tt_local), GFP_ATOMIC);
	if (!tt_local)
		goto out;

	batadv_dbg(BATADV_DBG_TT, bat_priv,
		   "Creating new local tt entry: %pM (ttvn: %d)\n", addr,
		   (uint8_t)atomic_read(&bat_priv->tt.vn));

	memcpy(tt_local->common.addr, addr, ETH_ALEN);
	/* The local entry has to be marked as NEW to avoid to send it in
	 * a full table response going out before the next ttvn increment
	 * (consistency check)
	 */
	tt_local->common.flags = BATADV_TT_CLIENT_NEW;
	if (batadv_is_wifi_iface(ifindex))
		tt_local->common.flags |= BATADV_TT_CLIENT_WIFI;
	atomic_set(&tt_local->common.refcount, 2);
	tt_local->last_seen = jiffies;
	tt_local->common.added_at = tt_local->last_seen;
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators

	spin_lock_bh(&bat_priv->tt_lhash_lock);

	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];

		rcu_read_lock();
		hlist_for_each_entry_rcu(tt_local_entry, node,
					 head, hash_entry) {
			if (buff_len < (count + 1) * ETH_ALEN)
				break;

			memcpy(buff + (count * ETH_ALEN), tt_local_entry->addr,
			       ETH_ALEN);

<<<<<<< HEAD
			count++;
=======
check_roaming:
	/* Check whether it is a roaming, but don't do anything if the roaming
	 * process has already been handled
	 */
	if (tt_global && !(tt_global->common.flags & BATADV_TT_CLIENT_ROAM)) {
		/* These node are probably going to update their tt table */
		head = &tt_global->orig_list;
		rcu_read_lock();
		hlist_for_each_entry_rcu(orig_entry, head, list) {
			batadv_send_roam_adv(bat_priv, tt_global->common.addr,
					     orig_entry->orig_node);
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
		}
		rcu_read_unlock();
	}

	/* if we did not get all new local tts see you next time  ;-) */
	if (count == bat_priv->num_local_tt)
		atomic_set(&bat_priv->tt_local_changed, 0);

	spin_unlock_bh(&bat_priv->tt_lhash_lock);
	return count;
}

int tt_local_seq_print_text(struct seq_file *seq, void *offset)
{
	struct net_device *net_dev = (struct net_device *)seq->private;
	struct bat_priv *bat_priv = netdev_priv(net_dev);
	struct hashtable_t *hash = bat_priv->tt_local_hash;
	struct tt_local_entry *tt_local_entry;
	struct hard_iface *primary_if;
	struct hlist_node *node;
	struct hlist_head *head;
	size_t buf_size, pos;
	char *buff;
	int i, ret = 0;

	primary_if = primary_if_get_selected(bat_priv);
	if (!primary_if) {
		ret = seq_printf(seq, "BATMAN mesh %s disabled - "
				 "please specify interfaces to enable it\n",
				 net_dev->name);
		goto out;
	}

	if (primary_if->if_status != IF_ACTIVE) {
		ret = seq_printf(seq, "BATMAN mesh %s disabled - "
				 "primary interface not active\n",
				 net_dev->name);
		goto out;
	}

	seq_printf(seq, "Locally retrieved addresses (from %s) "
		   "announced via TT:\n",
		   net_dev->name);

	spin_lock_bh(&bat_priv->tt_lhash_lock);

	buf_size = 1;
	/* Estimate length for: " * xx:xx:xx:xx:xx:xx\n" */
	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];

		rcu_read_lock();
		__hlist_for_each_rcu(node, head)
			buf_size += 21;
		rcu_read_unlock();
	}

<<<<<<< HEAD
	buff = kmalloc(buf_size, GFP_ATOMIC);
	if (!buff) {
		spin_unlock_bh(&bat_priv->tt_lhash_lock);
		ret = -ENOMEM;
=======
int batadv_tt_local_seq_print_text(struct seq_file *seq, void *offset)
{
	struct net_device *net_dev = (struct net_device *)seq->private;
	struct batadv_priv *bat_priv = netdev_priv(net_dev);
	struct batadv_hashtable *hash = bat_priv->tt.local_hash;
	struct batadv_tt_common_entry *tt_common_entry;
	struct batadv_tt_local_entry *tt_local;
	struct batadv_hard_iface *primary_if;
	struct hlist_head *head;
	uint32_t i;
	int last_seen_secs;
	int last_seen_msecs;
	unsigned long last_seen_jiffies;
	bool no_purge;
	uint16_t np_flag = BATADV_TT_CLIENT_NOPURGE;

	primary_if = batadv_seq_print_text_primary_if_get(seq);
	if (!primary_if)
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
		goto out;
	}

	buff[0] = '\0';
	pos = 0;

	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];

		rcu_read_lock();
<<<<<<< HEAD
		hlist_for_each_entry_rcu(tt_local_entry, node,
=======
		hlist_for_each_entry_rcu(tt_common_entry,
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
					 head, hash_entry) {
			pos += snprintf(buff + pos, 22, " * %pM\n",
					tt_local_entry->addr);
		}
		rcu_read_unlock();
	}

	spin_unlock_bh(&bat_priv->tt_lhash_lock);

	seq_printf(seq, "%s", buff);
	kfree(buff);
out:
	if (primary_if)
		hardif_free_ref(primary_if);
	return ret;
}

static void _tt_local_del(struct hlist_node *node, void *arg)
{
	struct bat_priv *bat_priv = (struct bat_priv *)arg;
	void *data = container_of(node, struct tt_local_entry, hash_entry);

	kfree(data);
	bat_priv->num_local_tt--;
	atomic_set(&bat_priv->tt_local_changed, 1);
}

static void tt_local_del(struct bat_priv *bat_priv,
			  struct tt_local_entry *tt_local_entry,
			  char *message)
{
	bat_dbg(DBG_ROUTES, bat_priv, "Deleting local tt entry (%pM): %s\n",
		tt_local_entry->addr, message);

	hash_remove(bat_priv->tt_local_hash, compare_ltt, choose_orig,
		    tt_local_entry->addr);
	_tt_local_del(&tt_local_entry->hash_entry, bat_priv);
}

void tt_local_remove(struct bat_priv *bat_priv,
		      uint8_t *addr, char *message)
{
<<<<<<< HEAD
	struct tt_local_entry *tt_local_entry;
=======
	struct batadv_tt_local_entry *tt_local_entry;
	struct batadv_tt_common_entry *tt_common_entry;
	struct hlist_node *node_tmp;

	hlist_for_each_entry_safe(tt_common_entry, node_tmp, head,
				  hash_entry) {
		tt_local_entry = container_of(tt_common_entry,
					      struct batadv_tt_local_entry,
					      common);
		if (tt_local_entry->common.flags & BATADV_TT_CLIENT_NOPURGE)
			continue;
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators

	spin_lock_bh(&bat_priv->tt_lhash_lock);

	tt_local_entry = tt_local_hash_find(bat_priv, addr);

	if (tt_local_entry)
		tt_local_del(bat_priv, tt_local_entry, message);

	spin_unlock_bh(&bat_priv->tt_lhash_lock);
}

static void tt_local_purge(struct work_struct *work)
{
<<<<<<< HEAD
	struct delayed_work *delayed_work =
		container_of(work, struct delayed_work, work);
	struct bat_priv *bat_priv =
		container_of(delayed_work, struct bat_priv, tt_work);
	struct hashtable_t *hash = bat_priv->tt_local_hash;
	struct tt_local_entry *tt_local_entry;
	struct hlist_node *node, *node_tmp;
=======
	struct batadv_hashtable *hash;
	spinlock_t *list_lock; /* protects write access to the hash lists */
	struct batadv_tt_common_entry *tt_common_entry;
	struct batadv_tt_local_entry *tt_local;
	struct hlist_node *node_tmp;
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
	struct hlist_head *head;
	unsigned long timeout;
	int i;

	spin_lock_bh(&bat_priv->tt_lhash_lock);

	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];

<<<<<<< HEAD
		hlist_for_each_entry_safe(tt_local_entry, node, node_tmp,
					  head, hash_entry) {
			if (tt_local_entry->never_purge)
				continue;
=======
		spin_lock_bh(list_lock);
		hlist_for_each_entry_safe(tt_common_entry, node_tmp,
					  head, hash_entry) {
			hlist_del_rcu(&tt_common_entry->hash_entry);
			tt_local = container_of(tt_common_entry,
						struct batadv_tt_local_entry,
						common);
			batadv_tt_local_entry_free_ref(tt_local);
		}
		spin_unlock_bh(list_lock);
	}

	batadv_hash_destroy(hash);

	bat_priv->tt.local_hash = NULL;
}

static int batadv_tt_global_init(struct batadv_priv *bat_priv)
{
	if (bat_priv->tt.global_hash)
		return 0;

	bat_priv->tt.global_hash = batadv_hash_new(1024);

	if (!bat_priv->tt.global_hash)
		return -ENOMEM;

	batadv_hash_set_lock_class(bat_priv->tt.global_hash,
				   &batadv_tt_global_hash_lock_class_key);

	return 0;
}
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators

			timeout = tt_local_entry->last_seen;
			timeout += TT_LOCAL_TIMEOUT * HZ;

			if (time_before(jiffies, timeout))
				continue;

			tt_local_del(bat_priv, tt_local_entry,
				      "address timed out");
		}
	}

	spin_unlock_bh(&bat_priv->tt_lhash_lock);
	tt_local_start_timer(bat_priv);
}

void tt_local_free(struct bat_priv *bat_priv)
{
<<<<<<< HEAD
	if (!bat_priv->tt_local_hash)
		return;
=======
	struct batadv_tt_orig_list_entry *tmp_orig_entry, *orig_entry = NULL;
	const struct hlist_head *head;

	rcu_read_lock();
	head = &entry->orig_list;
	hlist_for_each_entry_rcu(tmp_orig_entry, head, list) {
		if (tmp_orig_entry->orig_node != orig_node)
			continue;
		if (!atomic_inc_not_zero(&tmp_orig_entry->refcount))
			continue;

		orig_entry = tmp_orig_entry;
		break;
	}
	rcu_read_unlock();
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators

	cancel_delayed_work_sync(&bat_priv->tt_work);
	hash_delete(bat_priv->tt_local_hash, _tt_local_del, bat_priv);
	bat_priv->tt_local_hash = NULL;
}

int tt_global_init(struct bat_priv *bat_priv)
{
	if (bat_priv->tt_global_hash)
		return 1;

	bat_priv->tt_global_hash = hash_new(1024);

	if (!bat_priv->tt_global_hash)
		return 0;

	return 1;
}

void tt_global_add_orig(struct bat_priv *bat_priv,
			 struct orig_node *orig_node,
			 unsigned char *tt_buff, int tt_buff_len)
{
	struct tt_global_entry *tt_global_entry;
	struct tt_local_entry *tt_local_entry;
	int tt_buff_count = 0;
	unsigned char *tt_ptr;

	while ((tt_buff_count + 1) * ETH_ALEN <= tt_buff_len) {
		spin_lock_bh(&bat_priv->tt_ghash_lock);

		tt_ptr = tt_buff + (tt_buff_count * ETH_ALEN);
		tt_global_entry = tt_global_hash_find(bat_priv, tt_ptr);

		if (!tt_global_entry) {
			spin_unlock_bh(&bat_priv->tt_ghash_lock);

			tt_global_entry =
				kmalloc(sizeof(struct tt_global_entry),
					GFP_ATOMIC);

			if (!tt_global_entry)
				break;

			memcpy(tt_global_entry->addr, tt_ptr, ETH_ALEN);

			bat_dbg(DBG_ROUTES, bat_priv,
				"Creating new global tt entry: "
				"%pM (via %pM)\n",
				tt_global_entry->addr, orig_node->orig);

			spin_lock_bh(&bat_priv->tt_ghash_lock);
			hash_add(bat_priv->tt_global_hash, compare_gtt,
				 choose_orig, tt_global_entry,
				 &tt_global_entry->hash_entry);

		}

<<<<<<< HEAD
		tt_global_entry->orig_node = orig_node;
		spin_unlock_bh(&bat_priv->tt_ghash_lock);

		/* remove address from local hash if present */
		spin_lock_bh(&bat_priv->tt_lhash_lock);
=======
/* batadv_transtable_best_orig - Get best originator list entry from tt entry
 * @tt_global_entry: global translation table entry to be analyzed
 *
 * This functon assumes the caller holds rcu_read_lock().
 * Returns best originator list entry or NULL on errors.
 */
static struct batadv_tt_orig_list_entry *
batadv_transtable_best_orig(struct batadv_tt_global_entry *tt_global_entry)
{
	struct batadv_neigh_node *router = NULL;
	struct hlist_head *head;
	struct batadv_tt_orig_list_entry *orig_entry, *best_entry = NULL;
	int best_tq = 0;

	head = &tt_global_entry->orig_list;
	hlist_for_each_entry_rcu(orig_entry, head, list) {
		router = batadv_orig_node_get_router(orig_entry->orig_node);
		if (!router)
			continue;
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators

		tt_ptr = tt_buff + (tt_buff_count * ETH_ALEN);
		tt_local_entry = tt_local_hash_find(bat_priv, tt_ptr);

		if (tt_local_entry)
			tt_local_del(bat_priv, tt_local_entry,
				      "global tt received");

		spin_unlock_bh(&bat_priv->tt_lhash_lock);

<<<<<<< HEAD
		tt_buff_count++;
	}

	/* initialize, and overwrite if malloc succeeds */
	orig_node->tt_buff = NULL;
	orig_node->tt_buff_len = 0;
=======
/* batadv_tt_global_print_entry - print all orig nodes who announce the address
 * for this global entry
 * @tt_global_entry: global translation table entry to be printed
 * @seq: debugfs table seq_file struct
 *
 * This functon assumes the caller holds rcu_read_lock().
 */
static void
batadv_tt_global_print_entry(struct batadv_tt_global_entry *tt_global_entry,
			     struct seq_file *seq)
{
	struct hlist_head *head;
	struct batadv_tt_orig_list_entry *orig_entry, *best_entry;
	struct batadv_tt_common_entry *tt_common_entry;
	uint16_t flags;
	uint8_t last_ttvn;

	tt_common_entry = &tt_global_entry->common;
	flags = tt_common_entry->flags;

	best_entry = batadv_transtable_best_orig(tt_global_entry);
	if (best_entry) {
		last_ttvn = atomic_read(&best_entry->orig_node->last_ttvn);
		seq_printf(seq,
			   " %c %pM  (%3u) via %pM     (%3u)   (%#.4x) [%c%c%c]\n",
			   '*', tt_global_entry->common.addr,
			   best_entry->ttvn, best_entry->orig_node->orig,
			   last_ttvn, best_entry->orig_node->tt_crc,
			   (flags & BATADV_TT_CLIENT_ROAM ? 'R' : '.'),
			   (flags & BATADV_TT_CLIENT_WIFI ? 'W' : '.'),
			   (flags & BATADV_TT_CLIENT_TEMP ? 'T' : '.'));
	}

	head = &tt_global_entry->orig_list;

	hlist_for_each_entry_rcu(orig_entry, head, list) {
		if (best_entry == orig_entry)
			continue;
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators

	if (tt_buff_len > 0) {
		orig_node->tt_buff = kmalloc(tt_buff_len, GFP_ATOMIC);
		if (orig_node->tt_buff) {
			memcpy(orig_node->tt_buff, tt_buff, tt_buff_len);
			orig_node->tt_buff_len = tt_buff_len;
		}
	}
}

int tt_global_seq_print_text(struct seq_file *seq, void *offset)
{
	struct net_device *net_dev = (struct net_device *)seq->private;
<<<<<<< HEAD
	struct bat_priv *bat_priv = netdev_priv(net_dev);
	struct hashtable_t *hash = bat_priv->tt_global_hash;
	struct tt_global_entry *tt_global_entry;
	struct hard_iface *primary_if;
	struct hlist_node *node;
=======
	struct batadv_priv *bat_priv = netdev_priv(net_dev);
	struct batadv_hashtable *hash = bat_priv->tt.global_hash;
	struct batadv_tt_common_entry *tt_common_entry;
	struct batadv_tt_global_entry *tt_global;
	struct batadv_hard_iface *primary_if;
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
	struct hlist_head *head;
	size_t buf_size, pos;
	char *buff;
	int i, ret = 0;

	primary_if = primary_if_get_selected(bat_priv);
	if (!primary_if) {
		ret = seq_printf(seq, "BATMAN mesh %s disabled - please "
				 "specify interfaces to enable it\n",
				 net_dev->name);
		goto out;
	}

	if (primary_if->if_status != IF_ACTIVE) {
		ret = seq_printf(seq, "BATMAN mesh %s disabled - "
				 "primary interface not active\n",
				 net_dev->name);
		goto out;
	}

	seq_printf(seq,
		   "Globally announced TT entries received via the mesh %s\n",
		   net_dev->name);

	spin_lock_bh(&bat_priv->tt_ghash_lock);

	buf_size = 1;
	/* Estimate length for: " * xx:xx:xx:xx:xx:xx via xx:xx:xx:xx:xx:xx\n"*/
	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];

		rcu_read_lock();
		__hlist_for_each_rcu(node, head)
			buf_size += 43;
		rcu_read_unlock();
	}

	buff = kmalloc(buf_size, GFP_ATOMIC);
	if (!buff) {
		spin_unlock_bh(&bat_priv->tt_ghash_lock);
		ret = -ENOMEM;
		goto out;
	}
	buff[0] = '\0';
	pos = 0;

	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];

		rcu_read_lock();
<<<<<<< HEAD
		hlist_for_each_entry_rcu(tt_global_entry, node,
=======
		hlist_for_each_entry_rcu(tt_common_entry,
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
					 head, hash_entry) {
			pos += snprintf(buff + pos, 44,
					" * %pM via %pM\n",
					tt_global_entry->addr,
					tt_global_entry->orig_node->orig);
		}
		rcu_read_unlock();
	}

	spin_unlock_bh(&bat_priv->tt_ghash_lock);

	seq_printf(seq, "%s", buff);
	kfree(buff);
out:
	if (primary_if)
		hardif_free_ref(primary_if);
	return ret;
}

static void _tt_global_del_orig(struct bat_priv *bat_priv,
				 struct tt_global_entry *tt_global_entry,
				 char *message)
{
<<<<<<< HEAD
	bat_dbg(DBG_ROUTES, bat_priv,
		"Deleting global tt entry %pM (via %pM): %s\n",
		tt_global_entry->addr, tt_global_entry->orig_node->orig,
		message);

	hash_remove(bat_priv->tt_global_hash, compare_gtt, choose_orig,
		    tt_global_entry->addr);
	kfree(tt_global_entry);
=======
	struct hlist_head *head;
	struct hlist_node *safe;
	struct batadv_tt_orig_list_entry *orig_entry;

	spin_lock_bh(&tt_global_entry->list_lock);
	head = &tt_global_entry->orig_list;
	hlist_for_each_entry_safe(orig_entry, safe, head, list) {
		hlist_del_rcu(&orig_entry->list);
		batadv_tt_orig_list_entry_free_ref(orig_entry);
	}
	spin_unlock_bh(&tt_global_entry->list_lock);
}

static void
batadv_tt_global_del_orig_entry(struct batadv_priv *bat_priv,
				struct batadv_tt_global_entry *tt_global_entry,
				struct batadv_orig_node *orig_node,
				const char *message)
{
	struct hlist_head *head;
	struct hlist_node *safe;
	struct batadv_tt_orig_list_entry *orig_entry;

	spin_lock_bh(&tt_global_entry->list_lock);
	head = &tt_global_entry->orig_list;
	hlist_for_each_entry_safe(orig_entry, safe, head, list) {
		if (orig_entry->orig_node == orig_node) {
			batadv_dbg(BATADV_DBG_TT, bat_priv,
				   "Deleting %pM from global tt entry %pM: %s\n",
				   orig_node->orig,
				   tt_global_entry->common.addr, message);
			hlist_del_rcu(&orig_entry->list);
			batadv_tt_orig_list_entry_free_ref(orig_entry);
		}
	}
	spin_unlock_bh(&tt_global_entry->list_lock);
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
}

void tt_global_del_orig(struct bat_priv *bat_priv,
			 struct orig_node *orig_node, char *message)
{
<<<<<<< HEAD
	struct tt_global_entry *tt_global_entry;
	int tt_buff_count = 0;
	unsigned char *tt_ptr;
=======
	bool last_entry = true;
	struct hlist_head *head;
	struct batadv_tt_orig_list_entry *orig_entry;

	/* no local entry exists, case 1:
	 * Check if this is the last one or if other entries exist.
	 */

	rcu_read_lock();
	head = &tt_global_entry->orig_list;
	hlist_for_each_entry_rcu(orig_entry, head, list) {
		if (orig_entry->orig_node != orig_node) {
			last_entry = false;
			break;
		}
	}
	rcu_read_unlock();

	if (last_entry) {
		/* its the last one, mark for roaming. */
		tt_global_entry->common.flags |= BATADV_TT_CLIENT_ROAM;
		tt_global_entry->roam_at = jiffies;
	} else
		/* there is another entry, we can simply delete this
		 * one and can still use the other one.
		 */
		batadv_tt_global_del_orig_entry(bat_priv, tt_global_entry,
						orig_node, message);
}



static void batadv_tt_global_del(struct batadv_priv *bat_priv,
				 struct batadv_orig_node *orig_node,
				 const unsigned char *addr,
				 const char *message, bool roaming)
{
	struct batadv_tt_global_entry *tt_global_entry;
	struct batadv_tt_local_entry *local_entry = NULL;

	tt_global_entry = batadv_tt_global_hash_find(bat_priv, addr);
	if (!tt_global_entry)
		goto out;

	if (!roaming) {
		batadv_tt_global_del_orig_entry(bat_priv, tt_global_entry,
						orig_node, message);

		if (hlist_empty(&tt_global_entry->orig_list))
			batadv_tt_global_free(bat_priv, tt_global_entry,
					      message);

		goto out;
	}

	/* if we are deleting a global entry due to a roam
	 * event, there are two possibilities:
	 * 1) the client roamed from node A to node B => if there
	 *    is only one originator left for this client, we mark
	 *    it with BATADV_TT_CLIENT_ROAM, we start a timer and we
	 *    wait for node B to claim it. In case of timeout
	 *    the entry is purged.
	 *
	 *    If there are other originators left, we directly delete
	 *    the originator.
	 * 2) the client roamed to us => we can directly delete
	 *    the global entry, since it is useless now.
	 */
	local_entry = batadv_tt_local_hash_find(bat_priv,
						tt_global_entry->common.addr);
	if (local_entry) {
		/* local entry exists, case 2: client roamed to us. */
		batadv_tt_global_del_orig_list(tt_global_entry);
		batadv_tt_global_free(bat_priv, tt_global_entry, message);
	} else
		/* no local entry exists, case 1: check for roaming */
		batadv_tt_global_del_roaming(bat_priv, tt_global_entry,
					     orig_node, message);


out:
	if (tt_global_entry)
		batadv_tt_global_entry_free_ref(tt_global_entry);
	if (local_entry)
		batadv_tt_local_entry_free_ref(local_entry);
}

void batadv_tt_global_del_orig(struct batadv_priv *bat_priv,
			       struct batadv_orig_node *orig_node,
			       const char *message)
{
	struct batadv_tt_global_entry *tt_global;
	struct batadv_tt_common_entry *tt_common_entry;
	uint32_t i;
	struct batadv_hashtable *hash = bat_priv->tt.global_hash;
	struct hlist_node *safe;
	struct hlist_head *head;
	spinlock_t *list_lock; /* protects write access to the hash lists */
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators

	if (orig_node->tt_buff_len == 0)
		return;

<<<<<<< HEAD
	spin_lock_bh(&bat_priv->tt_ghash_lock);
=======
	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];
		list_lock = &hash->list_locks[i];

		spin_lock_bh(list_lock);
		hlist_for_each_entry_safe(tt_common_entry, safe,
					  head, hash_entry) {
			tt_global = container_of(tt_common_entry,
						 struct batadv_tt_global_entry,
						 common);

			batadv_tt_global_del_orig_entry(bat_priv, tt_global,
							orig_node, message);

			if (hlist_empty(&tt_global->orig_list)) {
				batadv_dbg(BATADV_DBG_TT, bat_priv,
					   "Deleting global tt entry %pM: %s\n",
					   tt_global->common.addr, message);
				hlist_del_rcu(&tt_common_entry->hash_entry);
				batadv_tt_global_entry_free_ref(tt_global);
			}
		}
		spin_unlock_bh(list_lock);
	}
	orig_node->tt_initialised = false;
}

static bool batadv_tt_global_to_purge(struct batadv_tt_global_entry *tt_global,
				      char **msg)
{
	bool purge = false;
	unsigned long roam_timeout = BATADV_TT_CLIENT_ROAM_TIMEOUT;
	unsigned long temp_timeout = BATADV_TT_CLIENT_TEMP_TIMEOUT;

	if ((tt_global->common.flags & BATADV_TT_CLIENT_ROAM) &&
	    batadv_has_timed_out(tt_global->roam_at, roam_timeout)) {
		purge = true;
		*msg = "Roaming timeout\n";
	}

	if ((tt_global->common.flags & BATADV_TT_CLIENT_TEMP) &&
	    batadv_has_timed_out(tt_global->common.added_at, temp_timeout)) {
		purge = true;
		*msg = "Temporary client timeout\n";
	}

	return purge;
}

static void batadv_tt_global_purge(struct batadv_priv *bat_priv)
{
	struct batadv_hashtable *hash = bat_priv->tt.global_hash;
	struct hlist_head *head;
	struct hlist_node *node_tmp;
	spinlock_t *list_lock; /* protects write access to the hash lists */
	uint32_t i;
	char *msg = NULL;
	struct batadv_tt_common_entry *tt_common;
	struct batadv_tt_global_entry *tt_global;

	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];
		list_lock = &hash->list_locks[i];

		spin_lock_bh(list_lock);
		hlist_for_each_entry_safe(tt_common, node_tmp, head,
					  hash_entry) {
			tt_global = container_of(tt_common,
						 struct batadv_tt_global_entry,
						 common);

			if (!batadv_tt_global_to_purge(tt_global, &msg))
				continue;

			batadv_dbg(BATADV_DBG_TT, bat_priv,
				   "Deleting global tt entry (%pM): %s\n",
				   tt_global->common.addr, msg);

			hlist_del_rcu(&tt_common->hash_entry);

			batadv_tt_global_entry_free_ref(tt_global);
		}
		spin_unlock_bh(list_lock);
	}
}

static void batadv_tt_global_table_free(struct batadv_priv *bat_priv)
{
	struct batadv_hashtable *hash;
	spinlock_t *list_lock; /* protects write access to the hash lists */
	struct batadv_tt_common_entry *tt_common_entry;
	struct batadv_tt_global_entry *tt_global;
	struct hlist_node *node_tmp;
	struct hlist_head *head;
	uint32_t i;

	if (!bat_priv->tt.global_hash)
		return;

	hash = bat_priv->tt.global_hash;

	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];
		list_lock = &hash->list_locks[i];

		spin_lock_bh(list_lock);
		hlist_for_each_entry_safe(tt_common_entry, node_tmp,
					  head, hash_entry) {
			hlist_del_rcu(&tt_common_entry->hash_entry);
			tt_global = container_of(tt_common_entry,
						 struct batadv_tt_global_entry,
						 common);
			batadv_tt_global_entry_free_ref(tt_global);
		}
		spin_unlock_bh(list_lock);
	}

	batadv_hash_destroy(hash);

	bat_priv->tt.global_hash = NULL;
}

static bool
_batadv_is_ap_isolated(struct batadv_tt_local_entry *tt_local_entry,
		       struct batadv_tt_global_entry *tt_global_entry)
{
	bool ret = false;

	if (tt_local_entry->common.flags & BATADV_TT_CLIENT_WIFI &&
	    tt_global_entry->common.flags & BATADV_TT_CLIENT_WIFI)
		ret = true;

	return ret;
}

struct batadv_orig_node *batadv_transtable_search(struct batadv_priv *bat_priv,
						  const uint8_t *src,
						  const uint8_t *addr)
{
	struct batadv_tt_local_entry *tt_local_entry = NULL;
	struct batadv_tt_global_entry *tt_global_entry = NULL;
	struct batadv_orig_node *orig_node = NULL;
	struct batadv_tt_orig_list_entry *best_entry;

	if (src && atomic_read(&bat_priv->ap_isolation)) {
		tt_local_entry = batadv_tt_local_hash_find(bat_priv, src);
		if (!tt_local_entry ||
		    (tt_local_entry->common.flags & BATADV_TT_CLIENT_PENDING))
			goto out;
	}

	tt_global_entry = batadv_tt_global_hash_find(bat_priv, addr);
	if (!tt_global_entry)
		goto out;

	/* check whether the clients should not communicate due to AP
	 * isolation
	 */
	if (tt_local_entry &&
	    _batadv_is_ap_isolated(tt_local_entry, tt_global_entry))
		goto out;

	rcu_read_lock();
	best_entry = batadv_transtable_best_orig(tt_global_entry);
	/* found anything? */
	if (best_entry)
		orig_node = best_entry->orig_node;
	if (orig_node && !atomic_inc_not_zero(&orig_node->refcount))
		orig_node = NULL;
	rcu_read_unlock();

out:
	if (tt_global_entry)
		batadv_tt_global_entry_free_ref(tt_global_entry);
	if (tt_local_entry)
		batadv_tt_local_entry_free_ref(tt_local_entry);

	return orig_node;
}

/* Calculates the checksum of the local table of a given orig_node */
static uint16_t batadv_tt_global_crc(struct batadv_priv *bat_priv,
				     struct batadv_orig_node *orig_node)
{
	uint16_t total = 0, total_one;
	struct batadv_hashtable *hash = bat_priv->tt.global_hash;
	struct batadv_tt_common_entry *tt_common;
	struct batadv_tt_global_entry *tt_global;
	struct hlist_head *head;
	uint32_t i;
	int j;

	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];

		rcu_read_lock();
		hlist_for_each_entry_rcu(tt_common, head, hash_entry) {
			tt_global = container_of(tt_common,
						 struct batadv_tt_global_entry,
						 common);
			/* Roaming clients are in the global table for
			 * consistency only. They don't have to be
			 * taken into account while computing the
			 * global crc
			 */
			if (tt_common->flags & BATADV_TT_CLIENT_ROAM)
				continue;
			/* Temporary clients have not been announced yet, so
			 * they have to be skipped while computing the global
			 * crc
			 */
			if (tt_common->flags & BATADV_TT_CLIENT_TEMP)
				continue;

			/* find out if this global entry is announced by this
			 * originator
			 */
			if (!batadv_tt_global_entry_has_orig(tt_global,
							     orig_node))
				continue;

			total_one = 0;
			for (j = 0; j < ETH_ALEN; j++)
				total_one = crc16_byte(total_one,
						       tt_common->addr[j]);
			total ^= total_one;
		}
		rcu_read_unlock();
	}

	return total;
}

/* Calculates the checksum of the local table */
static uint16_t batadv_tt_local_crc(struct batadv_priv *bat_priv)
{
	uint16_t total = 0, total_one;
	struct batadv_hashtable *hash = bat_priv->tt.local_hash;
	struct batadv_tt_common_entry *tt_common;
	struct hlist_head *head;
	uint32_t i;
	int j;

	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];

		rcu_read_lock();
		hlist_for_each_entry_rcu(tt_common, head, hash_entry) {
			/* not yet committed clients have not to be taken into
			 * account while computing the CRC
			 */
			if (tt_common->flags & BATADV_TT_CLIENT_NEW)
				continue;
			total_one = 0;
			for (j = 0; j < ETH_ALEN; j++)
				total_one = crc16_byte(total_one,
						       tt_common->addr[j]);
			total ^= total_one;
		}
		rcu_read_unlock();
	}

	return total;
}
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators

	while ((tt_buff_count + 1) * ETH_ALEN <= orig_node->tt_buff_len) {
		tt_ptr = orig_node->tt_buff + (tt_buff_count * ETH_ALEN);
		tt_global_entry = tt_global_hash_find(bat_priv, tt_ptr);

		if ((tt_global_entry) &&
		    (tt_global_entry->orig_node == orig_node))
			_tt_global_del_orig(bat_priv, tt_global_entry,
					     message);

		tt_buff_count++;
	}

	spin_unlock_bh(&bat_priv->tt_ghash_lock);

<<<<<<< HEAD
=======
	/* Replace the old buffer only if I received something in the
	 * last OGM (the OGM could carry no changes)
	 */
	spin_lock_bh(&orig_node->tt_buff_lock);
	if (tt_buff_len > 0) {
		kfree(orig_node->tt_buff);
		orig_node->tt_buff_len = 0;
		orig_node->tt_buff = kmalloc(tt_buff_len, GFP_ATOMIC);
		if (orig_node->tt_buff) {
			memcpy(orig_node->tt_buff, tt_buff, tt_buff_len);
			orig_node->tt_buff_len = tt_buff_len;
		}
	}
	spin_unlock_bh(&orig_node->tt_buff_lock);
}

static void batadv_tt_req_purge(struct batadv_priv *bat_priv)
{
	struct batadv_tt_req_node *node, *safe;

	spin_lock_bh(&bat_priv->tt.req_list_lock);
	list_for_each_entry_safe(node, safe, &bat_priv->tt.req_list, list) {
		if (batadv_has_timed_out(node->issued_at,
					 BATADV_TT_REQUEST_TIMEOUT)) {
			list_del(&node->list);
			kfree(node);
		}
	}
	spin_unlock_bh(&bat_priv->tt.req_list_lock);
}

/* returns the pointer to the new tt_req_node struct if no request
 * has already been issued for this orig_node, NULL otherwise
 */
static struct batadv_tt_req_node *
batadv_new_tt_req_node(struct batadv_priv *bat_priv,
		       struct batadv_orig_node *orig_node)
{
	struct batadv_tt_req_node *tt_req_node_tmp, *tt_req_node = NULL;

	spin_lock_bh(&bat_priv->tt.req_list_lock);
	list_for_each_entry(tt_req_node_tmp, &bat_priv->tt.req_list, list) {
		if (batadv_compare_eth(tt_req_node_tmp, orig_node) &&
		    !batadv_has_timed_out(tt_req_node_tmp->issued_at,
					  BATADV_TT_REQUEST_TIMEOUT))
			goto unlock;
	}

	tt_req_node = kmalloc(sizeof(*tt_req_node), GFP_ATOMIC);
	if (!tt_req_node)
		goto unlock;

	memcpy(tt_req_node->addr, orig_node->orig, ETH_ALEN);
	tt_req_node->issued_at = jiffies;

	list_add(&tt_req_node->list, &bat_priv->tt.req_list);
unlock:
	spin_unlock_bh(&bat_priv->tt.req_list_lock);
	return tt_req_node;
}

/* data_ptr is useless here, but has to be kept to respect the prototype */
static int batadv_tt_local_valid_entry(const void *entry_ptr,
				       const void *data_ptr)
{
	const struct batadv_tt_common_entry *tt_common_entry = entry_ptr;

	if (tt_common_entry->flags & BATADV_TT_CLIENT_NEW)
		return 0;
	return 1;
}

static int batadv_tt_global_valid(const void *entry_ptr,
				  const void *data_ptr)
{
	const struct batadv_tt_common_entry *tt_common_entry = entry_ptr;
	const struct batadv_tt_global_entry *tt_global_entry;
	const struct batadv_orig_node *orig_node = data_ptr;

	if (tt_common_entry->flags & BATADV_TT_CLIENT_ROAM ||
	    tt_common_entry->flags & BATADV_TT_CLIENT_TEMP)
		return 0;

	tt_global_entry = container_of(tt_common_entry,
				       struct batadv_tt_global_entry,
				       common);

	return batadv_tt_global_entry_has_orig(tt_global_entry, orig_node);
}

static struct sk_buff *
batadv_tt_response_fill_table(uint16_t tt_len, uint8_t ttvn,
			      struct batadv_hashtable *hash,
			      struct batadv_hard_iface *primary_if,
			      int (*valid_cb)(const void *, const void *),
			      void *cb_data)
{
	struct batadv_tt_common_entry *tt_common_entry;
	struct batadv_tt_query_packet *tt_response;
	struct batadv_tt_change *tt_change;
	struct hlist_head *head;
	struct sk_buff *skb = NULL;
	uint16_t tt_tot, tt_count;
	ssize_t tt_query_size = sizeof(struct batadv_tt_query_packet);
	uint32_t i;
	size_t len;

	if (tt_query_size + tt_len > primary_if->soft_iface->mtu) {
		tt_len = primary_if->soft_iface->mtu - tt_query_size;
		tt_len -= tt_len % sizeof(struct batadv_tt_change);
	}
	tt_tot = tt_len / sizeof(struct batadv_tt_change);

	len = tt_query_size + tt_len;
	skb = dev_alloc_skb(len + ETH_HLEN + NET_IP_ALIGN);
	if (!skb)
		goto out;

	skb_reserve(skb, ETH_HLEN + NET_IP_ALIGN);
	tt_response = (struct batadv_tt_query_packet *)skb_put(skb, len);
	tt_response->ttvn = ttvn;

	tt_change = (struct batadv_tt_change *)(skb->data + tt_query_size);
	tt_count = 0;

	rcu_read_lock();
	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];

		hlist_for_each_entry_rcu(tt_common_entry,
					 head, hash_entry) {
			if (tt_count == tt_tot)
				break;

			if ((valid_cb) && (!valid_cb(tt_common_entry, cb_data)))
				continue;

			memcpy(tt_change->addr, tt_common_entry->addr,
			       ETH_ALEN);
			tt_change->flags = tt_common_entry->flags;

			tt_count++;
			tt_change++;
		}
	}
	rcu_read_unlock();

	/* store in the message the number of entries we have successfully
	 * copied
	 */
	tt_response->tt_data = htons(tt_count);

out:
	return skb;
}

static int batadv_send_tt_request(struct batadv_priv *bat_priv,
				  struct batadv_orig_node *dst_orig_node,
				  uint8_t ttvn, uint16_t tt_crc,
				  bool full_table)
{
	struct sk_buff *skb = NULL;
	struct batadv_tt_query_packet *tt_request;
	struct batadv_hard_iface *primary_if;
	struct batadv_tt_req_node *tt_req_node = NULL;
	int ret = 1;
	size_t tt_req_len;

	primary_if = batadv_primary_if_get_selected(bat_priv);
	if (!primary_if)
		goto out;

	/* The new tt_req will be issued only if I'm not waiting for a
	 * reply from the same orig_node yet
	 */
	tt_req_node = batadv_new_tt_req_node(bat_priv, dst_orig_node);
	if (!tt_req_node)
		goto out;

	skb = dev_alloc_skb(sizeof(*tt_request) + ETH_HLEN + NET_IP_ALIGN);
	if (!skb)
		goto out;

	skb_reserve(skb, ETH_HLEN + NET_IP_ALIGN);

	tt_req_len = sizeof(*tt_request);
	tt_request = (struct batadv_tt_query_packet *)skb_put(skb, tt_req_len);

	tt_request->header.packet_type = BATADV_TT_QUERY;
	tt_request->header.version = BATADV_COMPAT_VERSION;
	memcpy(tt_request->src, primary_if->net_dev->dev_addr, ETH_ALEN);
	memcpy(tt_request->dst, dst_orig_node->orig, ETH_ALEN);
	tt_request->header.ttl = BATADV_TTL;
	tt_request->ttvn = ttvn;
	tt_request->tt_data = htons(tt_crc);
	tt_request->flags = BATADV_TT_REQUEST;

	if (full_table)
		tt_request->flags |= BATADV_TT_FULL_TABLE;

	batadv_dbg(BATADV_DBG_TT, bat_priv, "Sending TT_REQUEST to %pM [%c]\n",
		   dst_orig_node->orig, (full_table ? 'F' : '.'));

	batadv_inc_counter(bat_priv, BATADV_CNT_TT_REQUEST_TX);

	if (batadv_send_skb_to_orig(skb, dst_orig_node, NULL))
		ret = 0;

out:
	if (primary_if)
		batadv_hardif_free_ref(primary_if);
	if (ret)
		kfree_skb(skb);
	if (ret && tt_req_node) {
		spin_lock_bh(&bat_priv->tt.req_list_lock);
		list_del(&tt_req_node->list);
		spin_unlock_bh(&bat_priv->tt.req_list_lock);
		kfree(tt_req_node);
	}
	return ret;
}

static bool
batadv_send_other_tt_response(struct batadv_priv *bat_priv,
			      struct batadv_tt_query_packet *tt_request)
{
	struct batadv_orig_node *req_dst_orig_node;
	struct batadv_orig_node *res_dst_orig_node = NULL;
	struct batadv_hard_iface *primary_if = NULL;
	uint8_t orig_ttvn, req_ttvn, ttvn;
	int ret = false;
	unsigned char *tt_buff;
	bool full_table;
	uint16_t tt_len, tt_tot;
	struct sk_buff *skb = NULL;
	struct batadv_tt_query_packet *tt_response;
	uint8_t *packet_pos;
	size_t len;

	batadv_dbg(BATADV_DBG_TT, bat_priv,
		   "Received TT_REQUEST from %pM for ttvn: %u (%pM) [%c]\n",
		   tt_request->src, tt_request->ttvn, tt_request->dst,
		   (tt_request->flags & BATADV_TT_FULL_TABLE ? 'F' : '.'));

	/* Let's get the orig node of the REAL destination */
	req_dst_orig_node = batadv_orig_hash_find(bat_priv, tt_request->dst);
	if (!req_dst_orig_node)
		goto out;

	res_dst_orig_node = batadv_orig_hash_find(bat_priv, tt_request->src);
	if (!res_dst_orig_node)
		goto out;

	primary_if = batadv_primary_if_get_selected(bat_priv);
	if (!primary_if)
		goto out;

	orig_ttvn = (uint8_t)atomic_read(&req_dst_orig_node->last_ttvn);
	req_ttvn = tt_request->ttvn;

	/* I don't have the requested data */
	if (orig_ttvn != req_ttvn ||
	    tt_request->tt_data != htons(req_dst_orig_node->tt_crc))
		goto out;

	/* If the full table has been explicitly requested */
	if (tt_request->flags & BATADV_TT_FULL_TABLE ||
	    !req_dst_orig_node->tt_buff)
		full_table = true;
	else
		full_table = false;

	/* In this version, fragmentation is not implemented, then
	 * I'll send only one packet with as much TT entries as I can
	 */
	if (!full_table) {
		spin_lock_bh(&req_dst_orig_node->tt_buff_lock);
		tt_len = req_dst_orig_node->tt_buff_len;
		tt_tot = tt_len / sizeof(struct batadv_tt_change);

		len = sizeof(*tt_response) + tt_len;
		skb = dev_alloc_skb(len + ETH_HLEN + NET_IP_ALIGN);
		if (!skb)
			goto unlock;

		skb_reserve(skb, ETH_HLEN + NET_IP_ALIGN);
		packet_pos = skb_put(skb, len);
		tt_response = (struct batadv_tt_query_packet *)packet_pos;
		tt_response->ttvn = req_ttvn;
		tt_response->tt_data = htons(tt_tot);

		tt_buff = skb->data + sizeof(*tt_response);
		/* Copy the last orig_node's OGM buffer */
		memcpy(tt_buff, req_dst_orig_node->tt_buff,
		       req_dst_orig_node->tt_buff_len);

		spin_unlock_bh(&req_dst_orig_node->tt_buff_lock);
	} else {
		tt_len = (uint16_t)atomic_read(&req_dst_orig_node->tt_size);
		tt_len *= sizeof(struct batadv_tt_change);
		ttvn = (uint8_t)atomic_read(&req_dst_orig_node->last_ttvn);

		skb = batadv_tt_response_fill_table(tt_len, ttvn,
						    bat_priv->tt.global_hash,
						    primary_if,
						    batadv_tt_global_valid,
						    req_dst_orig_node);
		if (!skb)
			goto out;

		tt_response = (struct batadv_tt_query_packet *)skb->data;
	}

	tt_response->header.packet_type = BATADV_TT_QUERY;
	tt_response->header.version = BATADV_COMPAT_VERSION;
	tt_response->header.ttl = BATADV_TTL;
	memcpy(tt_response->src, req_dst_orig_node->orig, ETH_ALEN);
	memcpy(tt_response->dst, tt_request->src, ETH_ALEN);
	tt_response->flags = BATADV_TT_RESPONSE;

	if (full_table)
		tt_response->flags |= BATADV_TT_FULL_TABLE;

	batadv_dbg(BATADV_DBG_TT, bat_priv,
		   "Sending TT_RESPONSE %pM for %pM (ttvn: %u)\n",
		   res_dst_orig_node->orig, req_dst_orig_node->orig, req_ttvn);

	batadv_inc_counter(bat_priv, BATADV_CNT_TT_RESPONSE_TX);

	if (batadv_send_skb_to_orig(skb, res_dst_orig_node, NULL))
		ret = true;
	goto out;

unlock:
	spin_unlock_bh(&req_dst_orig_node->tt_buff_lock);

out:
	if (res_dst_orig_node)
		batadv_orig_node_free_ref(res_dst_orig_node);
	if (req_dst_orig_node)
		batadv_orig_node_free_ref(req_dst_orig_node);
	if (primary_if)
		batadv_hardif_free_ref(primary_if);
	if (!ret)
		kfree_skb(skb);
	return ret;
}

static bool
batadv_send_my_tt_response(struct batadv_priv *bat_priv,
			   struct batadv_tt_query_packet *tt_request)
{
	struct batadv_orig_node *orig_node;
	struct batadv_hard_iface *primary_if = NULL;
	uint8_t my_ttvn, req_ttvn, ttvn;
	int ret = false;
	unsigned char *tt_buff;
	bool full_table;
	uint16_t tt_len, tt_tot;
	struct sk_buff *skb = NULL;
	struct batadv_tt_query_packet *tt_response;
	uint8_t *packet_pos;
	size_t len;

	batadv_dbg(BATADV_DBG_TT, bat_priv,
		   "Received TT_REQUEST from %pM for ttvn: %u (me) [%c]\n",
		   tt_request->src, tt_request->ttvn,
		   (tt_request->flags & BATADV_TT_FULL_TABLE ? 'F' : '.'));


	my_ttvn = (uint8_t)atomic_read(&bat_priv->tt.vn);
	req_ttvn = tt_request->ttvn;

	orig_node = batadv_orig_hash_find(bat_priv, tt_request->src);
	if (!orig_node)
		goto out;

	primary_if = batadv_primary_if_get_selected(bat_priv);
	if (!primary_if)
		goto out;

	/* If the full table has been explicitly requested or the gap
	 * is too big send the whole local translation table
	 */
	if (tt_request->flags & BATADV_TT_FULL_TABLE || my_ttvn != req_ttvn ||
	    !bat_priv->tt.last_changeset)
		full_table = true;
	else
		full_table = false;

	/* In this version, fragmentation is not implemented, then
	 * I'll send only one packet with as much TT entries as I can
	 */
	if (!full_table) {
		spin_lock_bh(&bat_priv->tt.last_changeset_lock);
		tt_len = bat_priv->tt.last_changeset_len;
		tt_tot = tt_len / sizeof(struct batadv_tt_change);

		len = sizeof(*tt_response) + tt_len;
		skb = dev_alloc_skb(len + ETH_HLEN + NET_IP_ALIGN);
		if (!skb)
			goto unlock;

		skb_reserve(skb, ETH_HLEN + NET_IP_ALIGN);
		packet_pos = skb_put(skb, len);
		tt_response = (struct batadv_tt_query_packet *)packet_pos;
		tt_response->ttvn = req_ttvn;
		tt_response->tt_data = htons(tt_tot);

		tt_buff = skb->data + sizeof(*tt_response);
		memcpy(tt_buff, bat_priv->tt.last_changeset,
		       bat_priv->tt.last_changeset_len);
		spin_unlock_bh(&bat_priv->tt.last_changeset_lock);
	} else {
		tt_len = (uint16_t)atomic_read(&bat_priv->tt.local_entry_num);
		tt_len *= sizeof(struct batadv_tt_change);
		ttvn = (uint8_t)atomic_read(&bat_priv->tt.vn);

		skb = batadv_tt_response_fill_table(tt_len, ttvn,
						    bat_priv->tt.local_hash,
						    primary_if,
						    batadv_tt_local_valid_entry,
						    NULL);
		if (!skb)
			goto out;

		tt_response = (struct batadv_tt_query_packet *)skb->data;
	}

	tt_response->header.packet_type = BATADV_TT_QUERY;
	tt_response->header.version = BATADV_COMPAT_VERSION;
	tt_response->header.ttl = BATADV_TTL;
	memcpy(tt_response->src, primary_if->net_dev->dev_addr, ETH_ALEN);
	memcpy(tt_response->dst, tt_request->src, ETH_ALEN);
	tt_response->flags = BATADV_TT_RESPONSE;

	if (full_table)
		tt_response->flags |= BATADV_TT_FULL_TABLE;

	batadv_dbg(BATADV_DBG_TT, bat_priv,
		   "Sending TT_RESPONSE to %pM [%c]\n",
		   orig_node->orig,
		   (tt_response->flags & BATADV_TT_FULL_TABLE ? 'F' : '.'));

	batadv_inc_counter(bat_priv, BATADV_CNT_TT_RESPONSE_TX);

	if (batadv_send_skb_to_orig(skb, orig_node, NULL))
		ret = true;
	goto out;

unlock:
	spin_unlock_bh(&bat_priv->tt.last_changeset_lock);
out:
	if (orig_node)
		batadv_orig_node_free_ref(orig_node);
	if (primary_if)
		batadv_hardif_free_ref(primary_if);
	if (!ret)
		kfree_skb(skb);
	/* This packet was for me, so it doesn't need to be re-routed */
	return true;
}

bool batadv_send_tt_response(struct batadv_priv *bat_priv,
			     struct batadv_tt_query_packet *tt_request)
{
	if (batadv_is_my_mac(tt_request->dst)) {
		/* don't answer backbone gws! */
		if (batadv_bla_is_backbone_gw_orig(bat_priv, tt_request->src))
			return true;

		return batadv_send_my_tt_response(bat_priv, tt_request);
	} else {
		return batadv_send_other_tt_response(bat_priv, tt_request);
	}
}

static void _batadv_tt_update_changes(struct batadv_priv *bat_priv,
				      struct batadv_orig_node *orig_node,
				      struct batadv_tt_change *tt_change,
				      uint16_t tt_num_changes, uint8_t ttvn)
{
	int i;
	int roams;

	for (i = 0; i < tt_num_changes; i++) {
		if ((tt_change + i)->flags & BATADV_TT_CLIENT_DEL) {
			roams = (tt_change + i)->flags & BATADV_TT_CLIENT_ROAM;
			batadv_tt_global_del(bat_priv, orig_node,
					     (tt_change + i)->addr,
					     "tt removed by changes",
					     roams);
		} else {
			if (!batadv_tt_global_add(bat_priv, orig_node,
						  (tt_change + i)->addr,
						  (tt_change + i)->flags, ttvn))
				/* In case of problem while storing a
				 * global_entry, we stop the updating
				 * procedure without committing the
				 * ttvn change. This will avoid to send
				 * corrupted data on tt_request
				 */
				return;
		}
	}
	orig_node->tt_initialised = true;
}

static void batadv_tt_fill_gtable(struct batadv_priv *bat_priv,
				  struct batadv_tt_query_packet *tt_response)
{
	struct batadv_orig_node *orig_node;

	orig_node = batadv_orig_hash_find(bat_priv, tt_response->src);
	if (!orig_node)
		goto out;

	/* Purge the old table first.. */
	batadv_tt_global_del_orig(bat_priv, orig_node, "Received full table");

	_batadv_tt_update_changes(bat_priv, orig_node,
				  (struct batadv_tt_change *)(tt_response + 1),
				  ntohs(tt_response->tt_data),
				  tt_response->ttvn);

	spin_lock_bh(&orig_node->tt_buff_lock);
	kfree(orig_node->tt_buff);
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
	orig_node->tt_buff_len = 0;
	kfree(orig_node->tt_buff);
	orig_node->tt_buff = NULL;
}

static void tt_global_del(struct hlist_node *node, void *arg)
{
	void *data = container_of(node, struct tt_global_entry, hash_entry);

	kfree(data);
}

void tt_global_free(struct bat_priv *bat_priv)
{
<<<<<<< HEAD
	if (!bat_priv->tt_global_hash)
		return;

	hash_delete(bat_priv->tt_global_hash, tt_global_del, NULL);
	bat_priv->tt_global_hash = NULL;
=======
	struct batadv_tt_local_entry *tt_local_entry;
	bool ret = false;

	tt_local_entry = batadv_tt_local_hash_find(bat_priv, addr);
	if (!tt_local_entry)
		goto out;
	/* Check if the client has been logically deleted (but is kept for
	 * consistency purpose)
	 */
	if ((tt_local_entry->common.flags & BATADV_TT_CLIENT_PENDING) ||
	    (tt_local_entry->common.flags & BATADV_TT_CLIENT_ROAM))
		goto out;
	ret = true;
out:
	if (tt_local_entry)
		batadv_tt_local_entry_free_ref(tt_local_entry);
	return ret;
}

void batadv_handle_tt_response(struct batadv_priv *bat_priv,
			       struct batadv_tt_query_packet *tt_response)
{
	struct batadv_tt_req_node *node, *safe;
	struct batadv_orig_node *orig_node = NULL;
	struct batadv_tt_change *tt_change;

	batadv_dbg(BATADV_DBG_TT, bat_priv,
		   "Received TT_RESPONSE from %pM for ttvn %d t_size: %d [%c]\n",
		   tt_response->src, tt_response->ttvn,
		   ntohs(tt_response->tt_data),
		   (tt_response->flags & BATADV_TT_FULL_TABLE ? 'F' : '.'));

	/* we should have never asked a backbone gw */
	if (batadv_bla_is_backbone_gw_orig(bat_priv, tt_response->src))
		goto out;

	orig_node = batadv_orig_hash_find(bat_priv, tt_response->src);
	if (!orig_node)
		goto out;

	if (tt_response->flags & BATADV_TT_FULL_TABLE) {
		batadv_tt_fill_gtable(bat_priv, tt_response);
	} else {
		tt_change = (struct batadv_tt_change *)(tt_response + 1);
		batadv_tt_update_changes(bat_priv, orig_node,
					 ntohs(tt_response->tt_data),
					 tt_response->ttvn, tt_change);
	}

	/* Delete the tt_req_node from pending tt_requests list */
	spin_lock_bh(&bat_priv->tt.req_list_lock);
	list_for_each_entry_safe(node, safe, &bat_priv->tt.req_list, list) {
		if (!batadv_compare_eth(node->addr, tt_response->src))
			continue;
		list_del(&node->list);
		kfree(node);
	}
	spin_unlock_bh(&bat_priv->tt.req_list_lock);

	/* Recalculate the CRC for this orig_node and store it */
	orig_node->tt_crc = batadv_tt_global_crc(bat_priv, orig_node);
out:
	if (orig_node)
		batadv_orig_node_free_ref(orig_node);
}

int batadv_tt_init(struct batadv_priv *bat_priv)
{
	int ret;

	ret = batadv_tt_local_init(bat_priv);
	if (ret < 0)
		return ret;

	ret = batadv_tt_global_init(bat_priv);
	if (ret < 0)
		return ret;

	INIT_DELAYED_WORK(&bat_priv->tt.work, batadv_tt_purge);
	queue_delayed_work(batadv_event_workqueue, &bat_priv->tt.work,
			   msecs_to_jiffies(BATADV_TT_WORK_PERIOD));

	return 1;
}

static void batadv_tt_roam_list_free(struct batadv_priv *bat_priv)
{
	struct batadv_tt_roam_node *node, *safe;

	spin_lock_bh(&bat_priv->tt.roam_list_lock);

	list_for_each_entry_safe(node, safe, &bat_priv->tt.roam_list, list) {
		list_del(&node->list);
		kfree(node);
	}

	spin_unlock_bh(&bat_priv->tt.roam_list_lock);
}

static void batadv_tt_roam_purge(struct batadv_priv *bat_priv)
{
	struct batadv_tt_roam_node *node, *safe;

	spin_lock_bh(&bat_priv->tt.roam_list_lock);
	list_for_each_entry_safe(node, safe, &bat_priv->tt.roam_list, list) {
		if (!batadv_has_timed_out(node->first_time,
					  BATADV_ROAMING_MAX_TIME))
			continue;

		list_del(&node->list);
		kfree(node);
	}
	spin_unlock_bh(&bat_priv->tt.roam_list_lock);
}

/* This function checks whether the client already reached the
 * maximum number of possible roaming phases. In this case the ROAMING_ADV
 * will not be sent.
 *
 * returns true if the ROAMING_ADV can be sent, false otherwise
 */
static bool batadv_tt_check_roam_count(struct batadv_priv *bat_priv,
				       uint8_t *client)
{
	struct batadv_tt_roam_node *tt_roam_node;
	bool ret = false;

	spin_lock_bh(&bat_priv->tt.roam_list_lock);
	/* The new tt_req will be issued only if I'm not waiting for a
	 * reply from the same orig_node yet
	 */
	list_for_each_entry(tt_roam_node, &bat_priv->tt.roam_list, list) {
		if (!batadv_compare_eth(tt_roam_node->addr, client))
			continue;

		if (batadv_has_timed_out(tt_roam_node->first_time,
					 BATADV_ROAMING_MAX_TIME))
			continue;

		if (!batadv_atomic_dec_not_zero(&tt_roam_node->counter))
			/* Sorry, you roamed too many times! */
			goto unlock;
		ret = true;
		break;
	}

	if (!ret) {
		tt_roam_node = kmalloc(sizeof(*tt_roam_node), GFP_ATOMIC);
		if (!tt_roam_node)
			goto unlock;

		tt_roam_node->first_time = jiffies;
		atomic_set(&tt_roam_node->counter,
			   BATADV_ROAMING_MAX_COUNT - 1);
		memcpy(tt_roam_node->addr, client, ETH_ALEN);

		list_add(&tt_roam_node->list, &bat_priv->tt.roam_list);
		ret = true;
	}

unlock:
	spin_unlock_bh(&bat_priv->tt.roam_list_lock);
	return ret;
}

static void batadv_send_roam_adv(struct batadv_priv *bat_priv, uint8_t *client,
				 struct batadv_orig_node *orig_node)
{
	struct sk_buff *skb = NULL;
	struct batadv_roam_adv_packet *roam_adv_packet;
	int ret = 1;
	struct batadv_hard_iface *primary_if;
	size_t len = sizeof(*roam_adv_packet);

	/* before going on we have to check whether the client has
	 * already roamed to us too many times
	 */
	if (!batadv_tt_check_roam_count(bat_priv, client))
		goto out;

	skb = dev_alloc_skb(sizeof(*roam_adv_packet) + ETH_HLEN + NET_IP_ALIGN);
	if (!skb)
		goto out;

	skb_reserve(skb, ETH_HLEN + NET_IP_ALIGN);

	roam_adv_packet = (struct batadv_roam_adv_packet *)skb_put(skb, len);

	roam_adv_packet->header.packet_type = BATADV_ROAM_ADV;
	roam_adv_packet->header.version = BATADV_COMPAT_VERSION;
	roam_adv_packet->header.ttl = BATADV_TTL;
	roam_adv_packet->reserved = 0;
	primary_if = batadv_primary_if_get_selected(bat_priv);
	if (!primary_if)
		goto out;
	memcpy(roam_adv_packet->src, primary_if->net_dev->dev_addr, ETH_ALEN);
	batadv_hardif_free_ref(primary_if);
	memcpy(roam_adv_packet->dst, orig_node->orig, ETH_ALEN);
	memcpy(roam_adv_packet->client, client, ETH_ALEN);

	batadv_dbg(BATADV_DBG_TT, bat_priv,
		   "Sending ROAMING_ADV to %pM (client %pM)\n",
		   orig_node->orig, client);

	batadv_inc_counter(bat_priv, BATADV_CNT_TT_ROAM_ADV_TX);

	if (batadv_send_skb_to_orig(skb, orig_node, NULL))
		ret = 0;

out:
	if (ret && skb)
		kfree_skb(skb);
	return;
}

static void batadv_tt_purge(struct work_struct *work)
{
	struct delayed_work *delayed_work;
	struct batadv_priv_tt *priv_tt;
	struct batadv_priv *bat_priv;

	delayed_work = container_of(work, struct delayed_work, work);
	priv_tt = container_of(delayed_work, struct batadv_priv_tt, work);
	bat_priv = container_of(priv_tt, struct batadv_priv, tt);

	batadv_tt_local_purge(bat_priv);
	batadv_tt_global_purge(bat_priv);
	batadv_tt_req_purge(bat_priv);
	batadv_tt_roam_purge(bat_priv);

	queue_delayed_work(batadv_event_workqueue, &bat_priv->tt.work,
			   msecs_to_jiffies(BATADV_TT_WORK_PERIOD));
}

void batadv_tt_free(struct batadv_priv *bat_priv)
{
	cancel_delayed_work_sync(&bat_priv->tt.work);

	batadv_tt_local_table_free(bat_priv);
	batadv_tt_global_table_free(bat_priv);
	batadv_tt_req_list_free(bat_priv);
	batadv_tt_changes_list_free(bat_priv);
	batadv_tt_roam_list_free(bat_priv);

	kfree(bat_priv->tt.last_changeset);
}

/* This function will enable or disable the specified flags for all the entries
 * in the given hash table and returns the number of modified entries
 */
static uint16_t batadv_tt_set_flags(struct batadv_hashtable *hash,
				    uint16_t flags, bool enable)
{
	uint32_t i;
	uint16_t changed_num = 0;
	struct hlist_head *head;
	struct batadv_tt_common_entry *tt_common_entry;

	if (!hash)
		goto out;

	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];

		rcu_read_lock();
		hlist_for_each_entry_rcu(tt_common_entry,
					 head, hash_entry) {
			if (enable) {
				if ((tt_common_entry->flags & flags) == flags)
					continue;
				tt_common_entry->flags |= flags;
			} else {
				if (!(tt_common_entry->flags & flags))
					continue;
				tt_common_entry->flags &= ~flags;
			}
			changed_num++;
		}
		rcu_read_unlock();
	}
out:
	return changed_num;
}

/* Purge out all the tt local entries marked with BATADV_TT_CLIENT_PENDING */
static void batadv_tt_local_purge_pending_clients(struct batadv_priv *bat_priv)
{
	struct batadv_hashtable *hash = bat_priv->tt.local_hash;
	struct batadv_tt_common_entry *tt_common;
	struct batadv_tt_local_entry *tt_local;
	struct hlist_node *node_tmp;
	struct hlist_head *head;
	spinlock_t *list_lock; /* protects write access to the hash lists */
	uint32_t i;

	if (!hash)
		return;

	for (i = 0; i < hash->size; i++) {
		head = &hash->table[i];
		list_lock = &hash->list_locks[i];

		spin_lock_bh(list_lock);
		hlist_for_each_entry_safe(tt_common, node_tmp, head,
					  hash_entry) {
			if (!(tt_common->flags & BATADV_TT_CLIENT_PENDING))
				continue;

			batadv_dbg(BATADV_DBG_TT, bat_priv,
				   "Deleting local tt entry (%pM): pending\n",
				   tt_common->addr);

			atomic_dec(&bat_priv->tt.local_entry_num);
			hlist_del_rcu(&tt_common->hash_entry);
			tt_local = container_of(tt_common,
						struct batadv_tt_local_entry,
						common);
			batadv_tt_local_entry_free_ref(tt_local);
		}
		spin_unlock_bh(list_lock);
	}
}

static int batadv_tt_commit_changes(struct batadv_priv *bat_priv,
				    unsigned char **packet_buff,
				    int *packet_buff_len, int packet_min_len)
{
	uint16_t changed_num = 0;

	if (atomic_read(&bat_priv->tt.local_changes) < 1)
		return -ENOENT;

	changed_num = batadv_tt_set_flags(bat_priv->tt.local_hash,
					  BATADV_TT_CLIENT_NEW, false);

	/* all reset entries have to be counted as local entries */
	atomic_add(changed_num, &bat_priv->tt.local_entry_num);
	batadv_tt_local_purge_pending_clients(bat_priv);
	bat_priv->tt.local_crc = batadv_tt_local_crc(bat_priv);

	/* Increment the TTVN only once per OGM interval */
	atomic_inc(&bat_priv->tt.vn);
	batadv_dbg(BATADV_DBG_TT, bat_priv,
		   "Local changes committed, updating to ttvn %u\n",
		   (uint8_t)atomic_read(&bat_priv->tt.vn));

	/* reset the sending counter */
	atomic_set(&bat_priv->tt.ogm_append_cnt, BATADV_TT_OGM_APPEND_MAX);

	return batadv_tt_changes_fill_buff(bat_priv, packet_buff,
					   packet_buff_len, packet_min_len);
>>>>>>> b67bfe0... hlist: drop the node parameter from iterators
}

struct orig_node *transtable_search(struct bat_priv *bat_priv, uint8_t *addr)
{
	struct tt_global_entry *tt_global_entry;
	struct orig_node *orig_node = NULL;

	spin_lock_bh(&bat_priv->tt_ghash_lock);
	tt_global_entry = tt_global_hash_find(bat_priv, addr);

	if (!tt_global_entry)
		goto out;

	if (!atomic_inc_not_zero(&tt_global_entry->orig_node->refcount))
		goto out;

	orig_node = tt_global_entry->orig_node;

out:
	spin_unlock_bh(&bat_priv->tt_ghash_lock);
	return orig_node;
}
