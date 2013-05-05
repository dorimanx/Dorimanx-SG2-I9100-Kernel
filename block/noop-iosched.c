/*
 * elevator noop
 */
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>

struct noop_data {
	struct list_head queue;
	struct rb_root sort_list;
};

static void noop_dispatch_request(struct request_queue *, struct request *);

static void noop_add_rq_rb(struct noop_data *nd, struct request *rq)
{
	elv_rb_add(&nd->sort_list, rq);
}

static inline void noop_del_rq_rb(struct noop_data *nd, struct request *rq)
{
	elv_rb_del(&nd->sort_list, rq);
}

static int
noop_merge(struct request_queue *q, struct request **rqp, struct bio *bio)
{
	struct noop_data *nd = q->elevator->elevator_data;
	sector_t sector = bio->bi_sector + bio_sectors(bio);
	struct request *rq = elv_rb_find(&nd->sort_list, sector);

	if (rq && elv_rq_merge_ok(rq, bio)) {
		*rqp = rq;
		return ELEVATOR_FRONT_MERGE;
	}

	return ELEVATOR_NO_MERGE;
}

static void
noop_merged_request(struct request_queue *q, struct request *rq, int type)
{
	struct noop_data *nd = q->elevator->elevator_data;

	if (type == ELEVATOR_FRONT_MERGE) {
		noop_del_rq_rb(nd, rq);
		noop_add_rq_rb(nd, rq);
	}
}

static void noop_merged_requests(struct request_queue *q, struct request *rq,
				 struct request *next)
{
	struct noop_data *nd = q->elevator->elevator_data;

	list_del_init(&next->queuelist);
	noop_del_rq_rb(nd, next);
}

static void noop_add_request(struct request_queue *q, struct request *rq)
{
	struct noop_data *nd = q->elevator->elevator_data;

	list_add_tail(&rq->queuelist, &nd->queue);
	noop_add_rq_rb(nd, rq);
}

static void noop_dispatch_request(struct request_queue *q, struct request *rq)
{
	struct noop_data *nd = q->elevator->elevator_data;

	list_del_init(&rq->queuelist);
	noop_del_rq_rb(nd, rq);
	elv_dispatch_add_tail(q, rq);
}

static int noop_dispatch(struct request_queue *q, int force)
{
	struct noop_data *nd = q->elevator->elevator_data;

	if (!list_empty(&nd->queue)) {
		struct request *rq;
		rq = list_entry(nd->queue.next, struct request, queuelist);
		noop_dispatch_request(q, rq);
		return 1;
	}
	return 0;
}

static void *noop_init_queue(struct request_queue *q)
{
	struct noop_data *nd;

	nd = kmalloc_node(sizeof(*nd), GFP_KERNEL, q->node);
	if (!nd)
		return NULL;
	INIT_LIST_HEAD(&nd->queue);
	nd->sort_list = RB_ROOT;
	return nd;
}

static void noop_exit_queue(struct elevator_queue *e)
{
	struct noop_data *nd = e->elevator_data;

	BUG_ON(!list_empty(&nd->queue));
	kfree(nd);
}

static struct elevator_type elevator_noop = {
	.ops = {
		.elevator_merge_req_fn		= noop_merged_requests,
		.elevator_merge_fn              = noop_merge,
		.elevator_merged_fn             = noop_merged_request,
		.elevator_dispatch_fn		= noop_dispatch,
		.elevator_add_req_fn		= noop_add_request,
		.elevator_former_req_fn         = elv_rb_former_request,
		.elevator_latter_req_fn         = elv_rb_latter_request,
		.elevator_init_fn		= noop_init_queue,
		.elevator_exit_fn		= noop_exit_queue,
	},
	.elevator_name = "noop",
	.elevator_owner = THIS_MODULE,
};

static int __init noop_init(void)
{
	return elv_register(&elevator_noop);
}

static void __exit noop_exit(void)
{
	elv_unregister(&elevator_noop);
}

#ifdef CONFIG_FAST_RESUME
beforeresume_initcall(noop_init);
#else
module_init(noop_init);
#endif
module_exit(noop_exit);


MODULE_AUTHOR("Jens Axboe");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("No-op IO scheduler");

